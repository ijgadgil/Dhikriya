import os

from cogndn_core.context import (ContextItem, MemoryContextStore, RedisContextStore, digest,
                                 render_context, select_context)
from cogndn_core.grounding import canonical, check_provenance, extract_claims, ground
from cogndn_core.ledger import Ledger


def item(name, content, t=0.0, fresh=30.0, ok=True):
    return ContextItem(name, content, digest(canonical(content)), '/cogndn/authority/KEY/1', ok, t, t + fresh)


def test_memory_store_and_selection():
    st = MemoryContextStore()
    st.put(item('/cogndn/town/m3/hazard', {'category': 'animal', 'summary': 'livestock'}))
    st.put(item('/cogndn/town/m1/map', {'summary': 'tile'}, fresh=3600))
    st.put(item('/cogndn/town/m2/hazard', {'summary': 'old'}, t=-100, fresh=10))
    fresh = st.fresh_items(1.0)
    assert len(fresh) == 2
    sel = select_context(fresh, lambda z: {'m3': 40.0, 'm1': 0.0}.get(z), k=2)
    assert sel[0][0].name == '/cogndn/town/m3/hazard'           # hazard outranks a nearer map tile
    assert 'livestock' in render_context(sel, 2.0)[0]


def test_redis_store(redis_client):
    st = RedisContextStore(redis_client)
    ps = redis_client.pubsub()
    ps.subscribe('ctx_updates')
    ps.get_message(timeout=1)
    it = item('/cogndn/town/m3/hazard', {'category': 'animal'}, t=0.0, fresh=30.0)
    st.put(it, now=0.0)
    got = st.get(it.name)
    assert got.content == {'category': 'animal'} and got.sig_ok and got.digest == it.digest
    assert st.fresh_until(it.name) == 30.0
    assert [i.name for i in st.fresh_items(10.0, zones=['m3'])] == [it.name]
    assert st.fresh_items(31.0, zones=['m3']) == []
    msg = ps.get_message(timeout=1)
    assert msg and msg['data'] == it.name.encode()
    assert 0 < redis_client.pttl('ctx:' + it.name) <= 35000


def test_claims_and_grounding():
    assert extract_claims('Livestock ahead in m3, slowing down') == {'animal'}
    assert extract_claims('No pedestrians; road closed ahead, detour left') == {'closure'}
    ctx = [item('/cogndn/town/x1/detour', {'status': 'active'})]
    r = ground('Road closed ahead, taking the detour; cattle nearby', 'turn_left', [], ctx)
    assert r['support'] == {'animal': None, 'closure': 'data'}
    assert r['grounding'] == 0.5 and r['unsupported'] == ['animal']
    r = ground('Cow on the road', 'stop', [{'class': 'cow', 'confidence': 0.8}], [])
    assert r['grounding'] == 1.0 and r['support']['animal'] == 'perception'
    clear = [item('/cogndn/town/m3/hazard', {'category': 'animal', 'status': 'clear'})]
    assert ground('Cattle crossing ahead', 'slow_down', [], clear)['grounding'] == 0.0


def test_provenance_detects_tampering():
    st = MemoryContextStore()
    it = item('/cogndn/town/m3/hazard', {'category': 'animal'})
    st.put(it)
    cited = [{'name': it.name, 'digest': it.digest}, {'name': '/cogndn/town/zz/hazard', 'digest': 'x'}]
    assert [c['status'] for c in check_provenance(cited, st)] == ['ok', 'missing']
    st.tamper(it.name, {'category': 'none'})
    assert check_provenance(cited[:1], st)[0]['status'] == 'tampered'


def test_ledger_hash_chain(tmp_path):
    path = os.path.join(tmp_path, 'l.db')
    lg = Ledger(path)
    for i in range(5):
        lg.append(float(i), {'seq': i, 'action': 'keep_lane'})
    lg.commit()
    assert lg.verify() == (True, None)
    lg.con.execute("UPDATE records SET body = ? WHERE seq = 3", ('{"action":"stop"}',))
    lg.commit()
    assert lg.verify() == (False, 3)
    lg.close()
    lg2 = Ledger(path)
    assert lg2.seq == 5
