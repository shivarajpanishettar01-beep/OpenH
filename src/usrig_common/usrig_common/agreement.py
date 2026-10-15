"""Inter-rater agreement statistics (pure): percent agreement, Cohen's kappa (optionally weighted), Fleiss' kappa.

Landis & Koch (1977) bands are reported as a reading aid only; the acceptance threshold is the lab's choice.
"""
from collections import Counter


def percent_agreement(a, b):
    pairs = [(x, y) for x, y in zip(a, b) if x is not None and y is not None]
    return sum(x == y for x, y in pairs) / len(pairs) if pairs else None


def cohen_kappa(a, b, weights=None, categories=None):
    """Cohen's kappa for two raters over the same items (None = not rated, skipped).
    weights: None (nominal), 'linear' or 'quadratic' (ordinal categories, e.g. motion quality 1-5)."""
    pairs = [(x, y) for x, y in zip(a, b) if x is not None and y is not None]
    if not pairs:
        return None
    cats = list(categories) if categories else sorted({x for p in pairs for x in p}, key=str)
    k = len(cats)
    if k < 2:
        return 1.0 if all(x == y for x, y in pairs) else 0.0
    idx = {c: i for i, c in enumerate(cats)}
    n = len(pairs)

    def w(i, j):
        if weights is None:
            return 0.0 if i == j else 1.0
        d = abs(i - j) / (k - 1)
        return d if weights == 'linear' else d * d
    obs = sum(w(idx[x], idx[y]) for x, y in pairs) / n
    ca, cb = Counter(x for x, _ in pairs), Counter(y for _, y in pairs)
    exp = sum(w(idx[c1], idx[c2]) * ca[c1] * cb[c2] for c1 in cats for c2 in cats) / (n * n)
    if exp == 0:
        return 1.0
    return 1.0 - obs / exp


def fleiss_kappa(items):
    """items: list of per-item category lists (one entry per rater; every item rated by the same number of
    raters >= 2). Returns Fleiss' kappa, or None if there is nothing to compare."""
    items = [[x for x in it if x is not None] for it in items]
    items = [it for it in items if len(it) >= 2]
    if not items:
        return None
    m = len(items[0])
    if any(len(it) != m for it in items):
        raise ValueError('Fleiss kappa needs the same number of ratings per item')
    cats = sorted({x for it in items for x in it}, key=str)
    n = len(items)
    p_i = [(sum(c * c for c in Counter(it).values()) - m) / (m * (m - 1)) for it in items]
    p_bar = sum(p_i) / n
    p_j = [sum(Counter(it)[c] for it in items) / (n * m) for c in cats]
    pe = sum(p * p for p in p_j)
    return 1.0 if pe == 1 else (p_bar - pe) / (1 - pe)


def landis_koch(k):
    if k is None:
        return 'n/a'
    for lim, name in ((0.0, 'poor'), (0.2, 'slight'), (0.4, 'fair'), (0.6, 'moderate'), (0.8, 'substantial')):
        if k <= lim:
            return name
    return 'almost perfect'
