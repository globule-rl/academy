{- how evald, non-strictness
    bottom ⊥: in recursion, do not res in val, false
        fail w/ err, fail to terminate
        let x = x in x
    outside in: nested expr
        only apply to True -}
fst (0, undefined)
        == 0
possiblyKaboom = 
    \f -> f fst snd (0, undefined)
true :: a -> a -> a
true = \a -> (\b -> a)
false :: a -> a -> a
false = \a -> (\b -> b)
possiblyKaboom b = 
    case b of 
        True -> fst tup
        False -> snd tup
      where tup = (0, undefined)

(\f -> f fst snd (0, undefined)) (\a -> (\b -> a))
(\a -> (\b -> a)) fst snd (0, undefined)
(\b -> fst) snd (0, undefined)
fst (0, undefined)
0

{-  eval when binding used, not defined
        take list len w/o touching contents
    go: tail recursive pattern wrapper
        O(1) add x as advancing list
    const: ignores right args
        pointfree
        flip const: id ret snd arg
    expand foldr -> eval go -> eval const
    wc: week cons/:
    force eval: 
        case expr 
        create link
            y never evald, bottom never forces
                force: x -> y -> undefined
        seq
        bang -}
foldr k z xs = go xs
    where
        go []     = z
        go (y:ys) = y `k` go ys
c = foldr const 'z' ['a'..'e']
c = const 'z' "abcde" = go "abcde"
    where
        go []     = 'z'
        go ('a':"bcde") = 'a' `const` go "bcde"
const 'a' (go "bcde")
const x     y         = x
const 'a' (go "bcde") = 'a'
    const 'a' _ = 'a'
    foldr const 'z' ['a', undefined] == 'a'

onlySnd :: Int
onlySnd = 
    let x = undefined
        y = 2
        z = (x `seq` y `seq` 10, 11)
    in snd z
onlySnd
    == 11

hypo :: IO ()
hypo = do
    let x :: Int
        x = undefined
    s <- getLine
    case s of 
        "hi" -> print x
        _  -> putStrLn "hello"
hypo 
    == s hello
case x `seq` s of
hypo'
    == asd undefined
s <- x `seq` getLine
hypo''
    == undefined
seq :: a -> b -> b
seq bottom b    = bottom
seq notBottom b = b

let b = ???
case b of 
    True -> ...
    False

let wc x z = let y = undefined `seq` 'y' in x
foldr wc 'z' ['a'..'e']
       == 'a'
foldr (flip wc) 'z' ['a'..'e']
        == 'z'
let bot = undefined
let wc x = let y = bot `seq` 'y' in y `seq` x
foldr wc 'z' ['a'..'e']
    == undefined

{- seq weak head normal form whnf
    dc: data constructor
        whnf eval outer
        ignore undefined
    force eval: 
        case matching
        seq: nested
    determine strictness:
        inject bottoms
        core dump
    f $ x: apply f to x  -}
let dc = (,) undefined undefined
let noDc = undefined
let lam = \_ -> undefined
dc `seq` 1
    == 1
nodc `seq` 1
    == undefined
lam `seq` 1
    == 1

data Test = 
    A Test2
  | B Test2
  deriving (Show)
data Test2 =
    C Int
noForce :: Test -> Int
noForce _ = 0
noForce = \ _ -> I# 0#
forceTest :: Test -> Int
forceTest (A _) = 1
forceTest = 
    \ ds_d2oX ->
      case ds_d2oX of _ {
        A ds1_d2pI -> I# 1#;
        B ds1_d2pJ -> I# 2#;
      }
forceTest2 :: Test -> Int
forceTest2 (A (C i)) = i
forceTest2 =
    \ ds_d2n2 ->
      case ds_d2n2 of _ {
        A ds1_d2oV ->
            case ds1_d2oV of _ {
                C i_a1lo -> i_a1lo;
            }
      }
noForce undefined
    == 0
noForce (A undefined)
    == 0
forceTest (A undefined)
    == 1
forceTest undefined
    == undefined
forceTest2 (A (C 0))
    == 0
forceTest2 (A (C undefined))
    == undefined

coreDump :: Bool -> Int
coreDump b = 
    let x = undefined
    case b of
        False -> 0
        True -> 1
:set -ddump-simpl
:l code/coreDump.hs
:set -dsuppress-all
:r
coreDump :: Bool -> Int
[GblId, Arity=1, Caf=NoCafRefs, Str=DmdType]
coreDump =
    \ (b_aZJ :: Bool) ->
        case b_aZJ of _ [0cc=Dead] {
            False -> GHC.Types.I# 0;
            TRue -> GHC.Types.I# 1
        }
coreDump
coreDump = 
  \ b_aZY -> 
    case b_aZY of _ {
        False -> I# 0;
        True -> I# 1
    }

let x = undefined
in case x `seq` b of
    False -> 0
    True -> 1
let b_a10D ->
    let {
        x_a10E
        x_a10E = undefined } in
    case
        case x_a10E of _ {
            _DEFAULT -> b_a10D
        } of _ {
            False -> I# 0;
            True -> I# 1
        }
case undefined of { _ -> False }
case undefined of { DEFAULT -> False }

let myList = [undefined, 2, 3]
tail myList
    == [2, 3]
head $ sort [1, 2, 3, undefined]
    == undefined

{- trunk: ref later computaion, defer eval
    data constructor ::
        constant when applied
        func when not applied
    opportunity eval: no concrete type
        typeclass constraint eval
        id 1: eval to 1, compute
        (++): lambda func args passed to be evald -}
let myList = [1, 2, 3] :: [Integer]
:sprint myList
    == [1,2,3]
let myList2 = [1, 2, 3]
:t myList2
    myList2 :: Num t => [t]
:sprint myList2
    myList2 = _
let myList = [1, 2, id 1] :: [Integer]
:sprint myList
    myList = [1,2,_]
let myList' = myList ++ undefined
    myList' = _

import Debug.Trace
let a = trace "a" 1
let b = trace "b" 2
a + b
    == b a 3
inc = (+1)
twice = inc . inc
howManyTimes = 
    inc (trace 'evald' (1 + 1))
        + twice (trace 'evald' (1 + 1))
 howManyTimes' =
    let onePlusOne = trace 'evald' (1 + 1)
    in inc onePlusOne + twice onePlusOne
howManyTimes
    evald evald 7 
howManyTimes'
    evald 7     

{- sharing:
        same name 
        data constructor
    'a': Char literals
    "hello": String literals 
    string literal: optimization bytestrings, not list 
    prevent sharing:
        inline: create independent thunks
            make a func instead of f = 1
            two occurance not the same name
        func explicit named args
            non-strict, dont remember res, not fully lazy
        typeclass constraints: func
            wait to become concrete type, eval a twice
            concrete type: sharing 
            fm':: Num b => Maybe Int
        implicit parameter
        polymorphic 
            fm :: Num b => Maybe b
            reeval Just 1 no sharing
        prevent mem hanging
        add named arg: 
            const pointfree sharing
        force sharing: give name
            let
         -}
let a = trace "a" (1 :: Int)
a + a
    -> a 2
        -> 2
let a = Just ['a']
:sprint a
    a = Just "a"
returnIO 
    (: ((Just (: (C# 'a') ([])))
        `cast` ...) ([]))
let a = Just "a"
    a = Just _
returnIO 
    (: ((Just (unpackCString# "a"#))
        `cast` ...) ([]))
let a :: Int; a = trace "a" 2 + 2
let b = (a + a)
b
    a 8
b
    8

let f :: a -> Int; f _ = trace "f" 1
f 'a'
    f 1
:{let c :: Int;
    c = (trace "a" 2 + 2) + (trace "a" 2 + 2):}
c
    a a 8
let f :: a -> Int; f = trace "f" const 1
f 'a'
    f 1
f 'b'
    1

fmap ((+1) :: Int -> Int) blah
    Just 2
    blah = _
    blah :: Num a => Maybe a
let bl = Just 1
    bl :: Num a => Maybe a
    bl = _
fmap (+1) bl
    Just 2
let fm = fmap (+1) bl
    fm :: Num b => Maybe b
    fm = _
    Just 2
    fm = _
let fm' = fmap ((+1) :: Int -> Int) blah
    fm':: Num b => Maybe Int
    Just evald 1
    2
    fm' = Just 2

let add :: (?x :: Int) => Int
    add = trace "add" 1 + ?x
let ?x = 1 in add
    add 2

let blah = Just (trace "evald 1" 1)
    blah = _
    blah :: Num a => Maybe a
fmap (+1) blah
    Just evald 1
    2
    blah = _
let blah = Just (trace "evald 1" (1 :: Int))
    blah = Just _
fmap (+1) blah
    Just 2

let poly = 1
let conc = poly :: Int
    poly = _
    conc = _
conc
    1
    conc = 1

let f x = x + x
    hi 4
let f x = (x ()) + (x ())
f (\_ -> trace "hi" 2)
    hi hi 2
let f x = (x 2) + (x 10)
    hi hi 14
let g = const (trace "hi" 2)
f g
    hi 4
let g = \_ -> trace "hi" 2
f g
    hi hi 4

let x = 1 + 1
in x * x
forever :: (Monad m) => m a -> m b
forever a = let a' = a >> a' in a

{- refutable pattern: potential failure
    irrefutable pattern:
        never fail to match
        lazy pattern -}
refutable :: Bool -> Bool
refutable True = False
refutable False = True
irrefutable :: Bool -> Bool
irrefutable x = not x
oneOfEach :: Bool -> Bool
oneOfEach True = False
oneOfEach _ = True
isItTwo :: Integer -> Bool
isItTwo 2 = True
isItTwo _ = False

strictPattern :: (a, b) -> String
strictPattern (a,b) = const "lazy" a
lazyPattern :: (a, b) -> String
lazyPattern ~(a,b) = const "lazy" a
strictPattern undefined
    undefined
lazyPattern undefined
    "lazy"

noEval :: Bool -> Int
noEval b = 1
    \ _ -> I# 1#
seqEval :: Bool -> Int
seqEval b = b `seq` 1
    \ b_alia -> 
      case b_alia of _
        {__DEFAULT -> I# 1#}
bangPattern !b = 1
    \ b_alib -> 
      case b_alib of _
        {__DEFAULT -> I# 1#}
data Foo = Foo Int !Int
fst (Foo x _) = x
snd (Foo _ y) = y
snd (Foo undefined 1)
    1
fst (Foo 1 undefined)
    undefined

blah x = 1
main = print (blah undefined)
blah x = x `seq` 1
blah !x = 1
notForce ~x = 1
notForce undefined
    1

{- sTake n _ | n <= 0 = Nil
    sTake 0 _ = Nil
    add strictness
        a -> !a
        List a -> !(List a) !xs -}
data List a =
    Nil
    | Cons a (List a) deriving Show
sTake :: Int -> List a -> List a
sTake n _
    | n <= 0 = Nil
sTake n Nil = Nil
sTake n (Cons x xs) = (Cons x (sTake (n-1) xs))
twoEles = Cons 1 (Cons undefined Nil)
    == undefined
oneEle = sTake 1 twoEles
    == sTake 1 (Cons 1 (Cons undefined Nil))
    == Cons 1 Nil

| Cons !a (List a) deriving Show 
threeEles = Cons 2 twoEles
        == Cons 2 (Cons 1** undefined)
oneEleT = sTake 1 threeEles
        == sTake 1 (Cons 2 (Cons 1 (Cons undefined Nil))) 
        == Cons 2 Nil

| Cons !a !(List a) deriving Show
oneEle -> undefined