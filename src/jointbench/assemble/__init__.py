"""Level 4a, "assembles": the answer is two parts that bolt to each other.

``generate`` makes a task (facing mounts with a closed frame between them,
so the connection cannot be one part) and its hidden two-part reference;
``grade`` judges an answer: each part by the existing checks, then the way
the parts go in (``sweep``), their joint and its bolts (``joint``), and the
stated load on the two together. See docs/notes/level4.md.
"""
