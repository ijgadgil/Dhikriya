"""Segmentation of large objects: /<name>/seg=k, every segment signed, FinalBlockId on each.
NDN packets are limited to about 8800 bytes, so map tiles and detour maps must be segmented."""
from ndn.encoding import Component, MetaInfo, Name, make_data, parse_data

SEG_SIZE = 7000


def object_name(name):
    """Strip a trailing segment component from an encoded or string name; returns the string name."""
    n = Name.from_str(name) if isinstance(name, str) else list(name)
    if n and Component.get_type(n[-1]) == Component.TYPE_SEGMENT:
        n = n[:-1]
    return Name.to_str(n)


def segment_count(content_bytes, seg_size=SEG_SIZE):
    return max(1, -(-len(content_bytes) // seg_size))


def make_segment(name, content_bytes, k, freshness_s, signer, seg_size=SEG_SIZE):
    last = segment_count(content_bytes, seg_size) - 1
    seg_name = Name.from_str(name) + [Component.from_segment(k)]
    meta = MetaInfo(freshness_period=int(freshness_s * 1000), final_block_id=Component.from_segment(last))
    return make_data(seg_name, meta, content_bytes[k * seg_size:(k + 1) * seg_size], signer=signer)


def parse_segment(wire):
    name, meta, content, sig = parse_data(wire, with_tl=True)
    k = Component.to_number(name[-1])
    last = Component.to_number(meta.final_block_id) if meta.final_block_id else k
    return Name.to_str(name[:-1]), k, last, meta, bytes(content or b''), sig
