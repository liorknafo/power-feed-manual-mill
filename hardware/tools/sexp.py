"""Minimal S-expression reader/writer for KiCad files."""


class Sym(str):
    """Unquoted atom."""


def parse(text):
    tokens = _tokenize(text)
    pos = 0

    def read():
        nonlocal pos
        tok = tokens[pos]
        pos += 1
        if tok == "(":
            lst = []
            while tokens[pos] != ")":
                lst.append(read())
            pos += 1
            return lst
        return tok

    out = []
    while pos < len(tokens):
        out.append(read())
    return out[0] if len(out) == 1 else out


def _tokenize(text):
    toks, i, n = [], 0, len(text)
    while i < n:
        c = text[i]
        if c in " \t\r\n":
            i += 1
        elif c in "()":
            toks.append(c)
            i += 1
        elif c == '"':
            j = i + 1
            buf = []
            while text[j] != '"':
                if text[j] == "\\":
                    buf.append(text[j + 1])
                    j += 2
                else:
                    buf.append(text[j])
                    j += 1
            toks.append(str(''.join(buf)))  # quoted string -> plain str
            i = j + 1
        else:
            j = i
            while j < n and text[j] not in " \t\r\n()":
                j += 1
            toks.append(Sym(text[i:j]))
            i = j
    return toks


def dump(node, indent=0):
    pad = "\t" * indent
    if isinstance(node, list):
        if not node:
            return pad + "()"
        simple = all(not isinstance(x, list) for x in node)
        if simple:
            return pad + "(" + " ".join(_atom(x) for x in node) + ")"
        parts = [pad + "(" + _atom(node[0])]
        rest = node[1:]
        # keep leading atoms on the head line
        while rest and not isinstance(rest[0], list):
            parts[0] += " " + _atom(rest[0])
            rest = rest[1:]
        for x in rest:
            parts.append(dump(x, indent + 1))
        parts.append(pad + ")")
        return "\n".join(parts)
    return pad + _atom(node)


def _atom(x):
    if isinstance(x, Sym):
        return str(x)
    if isinstance(x, bool):
        return "yes" if x else "no"
    if isinstance(x, (int, float)):
        return _num(x)
    s = str(x).replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n")
    return '"' + s + '"'


def _num(v):
    if isinstance(v, int):
        return str(v)
    s = ("%.4f" % v).rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def find(node, key):
    """First child list whose head is `key`."""
    for x in node:
        if isinstance(x, list) and x and x[0] == key:
            return x
    return None


def find_all(node, key):
    return [x for x in node if isinstance(x, list) and x and x[0] == key]
