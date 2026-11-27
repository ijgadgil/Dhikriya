"""Create the authority key pairs used by producers and the consumer's trust policy.
  ros2 run cogndn_ndn make_keys            (keys in ~/.cogndn/keys)"""
import argparse
import os

from .paths import trust_path
from .security import make_keys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--keys', default=os.path.expanduser('~/.cogndn/keys'))
    a = ap.parse_args()
    names = make_keys(a.keys, trust_path())
    for r, n in names.items():
        print('%-13s %s' % (r, n))
    print('keys written to', a.keys)


if __name__ == '__main__':
    main()
