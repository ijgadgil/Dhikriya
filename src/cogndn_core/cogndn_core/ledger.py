"""Append-only, hash-chained explanation ledger (SQLite, WAL mode).

Each record stores h_n = SHA-256(h_{n-1} || canonical(body_n)), with h_0 = 64 zeros.
Changing, deleting or reordering any stored record breaks the chain from that point on.
"""
import hashlib
import json
import sqlite3

GENESIS = '0' * 64


def _hash(prev, body):
    return hashlib.sha256((prev + body).encode()).hexdigest()


class Ledger:
    def __init__(self, path):
        self.con = sqlite3.connect(path, check_same_thread=False)
        self.con.execute('PRAGMA journal_mode=WAL')
        self.con.execute('CREATE TABLE IF NOT EXISTS records(seq INTEGER PRIMARY KEY, t REAL, body TEXT, '
                         'prev_hash TEXT, hash TEXT)')
        row = self.con.execute('SELECT seq, hash FROM records ORDER BY seq DESC LIMIT 1').fetchone()
        self.seq, self.head = (row[0], row[1]) if row else (0, GENESIS)

    def append(self, t, record):
        body = json.dumps(record, sort_keys=True, separators=(',', ':'))
        self.seq += 1
        h = _hash(self.head, body)
        self.con.execute('INSERT INTO records VALUES (?,?,?,?,?)', (self.seq, t, body, self.head, h))
        self.head = h
        return self.seq, h

    def commit(self):
        self.con.commit()

    def records(self):
        return [(s, t, json.loads(b), p, h) for s, t, b, p, h in
                self.con.execute('SELECT seq, t, body, prev_hash, hash FROM records ORDER BY seq')]

    def verify(self):
        """Returns (True, None) if intact, else (False, first_bad_seq)."""
        prev = GENESIS
        expected = 1
        for s, _, b, p, h in self.con.execute('SELECT seq, t, body, prev_hash, hash FROM records ORDER BY seq'):
            if s != expected or p != prev or _hash(prev, b) != h:
                return False, s
            prev, expected = h, s + 1
        return True, None

    def close(self):
        self.con.commit()
        self.con.close()
