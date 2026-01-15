-- referential transparency: same inp, out same predictable
-- abstraction: shorter, repeated structures, generic reused
-- function: relation map, inp out set range, could be same out
## lambda calculus: left accociate
## lambda: head, 
    expressions:
        var
        abstraction-func, head(lamda+var/arg).body 𝜆𝑥.𝑥
## beta β-reduction: apply func by substituing arg
    replace bound var(x) with arg, eliminate head
    (𝜆𝑥.𝑥)2 -> 2 identity func
    (𝜆𝑥.𝑥+1) body->(x+1)
    (𝜆𝑥.𝑥)(𝜆𝑦.𝑦) -> 𝜆𝑦.𝑦
    (𝜆𝑥.𝑥)(𝜆𝑦.𝑦)𝑧 -> ((𝜆𝑥.𝑥)(𝜆𝑦.𝑦))𝑧 -> z
## free var: var in the body not bound by the head
    (𝜆𝑥.𝑥𝑦)z -> zy
## equivalence: 𝜆𝑥𝑦.𝑦𝑥 <=> 𝜆𝑎𝑏.𝑏𝑎
## only accept one arg, multi being shortcuts: 
    𝜆𝑥.(𝜆𝑦.𝑥𝑦) ->> 𝜆𝑥𝑦.𝑥𝑦 
        apply first arg, bind x, eliminate outer lambda(𝜆𝑥?) 
        have 𝜆𝑦.𝑥𝑦 with x being what the outer lambda bound to(𝜆𝑥(𝜆𝑦).xy->𝜆𝑥𝑦.xy?)
    (𝜆𝑥𝑦.𝑥𝑦)12 -> (𝜆𝑥(𝜆𝑦).𝑥𝑦)12 -> ((𝜆𝑦).1𝑦)2 -> 12
    (𝜆𝑥𝑦.𝑥𝑦)(𝜆𝑧.𝑎)1 ->(two arg: 𝜆𝑧.𝑎 & 1)-> (𝜆𝑥(𝜆𝑦).𝑥𝑦)(𝜆𝑧.𝑎)1 ->apply arg (𝜆𝑧.𝑎) as x
    ->((𝜆𝑦).(𝜆𝑧.𝑎)𝑦)1 -> (𝜆𝑧.𝑎)1 -> a
    (𝜆𝑥𝑦𝑧.𝑥𝑧(𝑦𝑧))(𝜆𝑚𝑛.𝑚)(𝜆𝑝.𝑝)->(𝜆𝑥.𝜆𝑦.𝜆𝑧.𝑥𝑧(𝑦𝑧))(𝜆𝑚.𝜆𝑛.𝑚)(𝜆𝑝.𝑝)->1st arg(𝜆𝑚.𝜆𝑛.𝑚), bind x to
    ->(𝜆𝑦.𝜆𝑧.(𝜆𝑚.𝜆𝑛.𝑚)𝑧(𝑦𝑧))(𝜆𝑝.𝑝)->arg (𝜆𝑝.𝑝), bind y to
    ->𝜆𝑧.(𝜆𝑚.𝜆𝑛.𝑚)(𝑧)((𝜆𝑝.𝑝)𝑧)->𝑧 irrereducible, no arg to apply to, go inside m
    ->𝜆𝑧.(𝜆𝑛.𝑧)((𝜆𝑝.𝑝)𝑧)->arg ((𝜆𝑝.𝑝)𝑧)->z
    ->𝜆𝑧.(𝜆𝑛.𝑧)𝑧 -> bind n, leftmost reducible 𝜆𝑛.𝑧->z
    ->𝜆𝑧.𝑧 identity func takes z ret z
## evaluation: simplification -> beta normal form
## combinator: 𝜆𝑦.𝑥 y is bound, x is free
## divergence: reduction no ending -> (𝜆𝑥.𝑥𝑥)(𝜆𝑥.𝑥𝑥) omega diverge
