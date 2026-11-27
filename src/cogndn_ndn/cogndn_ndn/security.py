"""ECDSA P-256 keys, a type-based trust policy and verification of signed Data.

Each data type may only be signed by one authority (config/trust.yaml). A Data packet is
accepted when (1) its KeyLocator names the authority allowed for its type and (2) the ECDSA
signature verifies with that authority's public key. Anything else is rejected and logged.
"""
import os

import yaml
from Cryptodome.PublicKey import ECC
from ndn.encoding import Name
from ndn.security.signer.sha256_ecdsa_signer import Sha256WithEcdsaSigner
from ndn.security.validator.known_key_validator import verify_ecdsa

from cogndn_core.naming import parse_name


def make_keys(key_dir, trust_path, roles=None, attacker=True):
    """Create one key pair per authority (and an 'attacker' key for forgery tests)."""
    os.makedirs(key_dir, exist_ok=True)
    cfg = yaml.safe_load(open(trust_path))
    names = dict((r, a['key_name']) for r, a in cfg['authorities'].items() if roles is None or r in roles)
    if attacker:
        names['attacker'] = '/cogndn/authority/road/KEY/666'
    for role in names:
        k = ECC.generate(curve='P-256')
        with open(os.path.join(key_dir, role + '.key.der'), 'wb') as f:
            f.write(k.export_key(format='DER'))
        with open(os.path.join(key_dir, role + '.pub.der'), 'wb') as f:
            f.write(k.public_key().export_key(format='DER'))
    return names


class TrustPolicy:
    def __init__(self, trust_path, key_dir):
        cfg = yaml.safe_load(open(trust_path))
        self.key_dir = key_dir
        self.by_type, self.pub, self.key_name = {}, {}, {}
        for role, a in cfg['authorities'].items():
            self.key_name[role] = a['key_name']
            for t in a['types']:
                self.by_type[t] = role
            p = os.path.join(key_dir, role + '.pub.der')
            if os.path.exists(p):
                self.pub[a['key_name']] = ECC.import_key(open(p, 'rb').read())

    def signer_for(self, dtype, role_override=None):
        role = role_override or self.by_type[dtype]
        key = open(os.path.join(self.key_dir, role + '.key.der'), 'rb').read()
        kn = self.key_name.get(role, '/cogndn/authority/road/KEY/666')
        return Sha256WithEcdsaSigner(kn, key), kn

    def verify(self, name, sig_ptrs):
        """Returns (ok, signer_key_name, reason)."""
        try:
            dtype = parse_name(name if isinstance(name, str) else Name.to_str(name))['type']
        except ValueError:
            return False, '', 'foreign name'
        kl = sig_ptrs.signature_info.key_locator if sig_ptrs.signature_info else None
        if kl is None or kl.name is None:
            return False, '', 'no key locator'
        kn = Name.to_str(kl.name)
        allowed = self.key_name.get(self.by_type.get(dtype, ''), None)
        if kn != allowed:
            return False, kn, 'signer %s not allowed for %s' % (kn, dtype)
        pub = self.pub.get(kn)
        if pub is None:
            return False, kn, 'unknown key'
        if not verify_ecdsa(pub, sig_ptrs):
            return False, kn, 'bad signature'
        return True, kn, ''
