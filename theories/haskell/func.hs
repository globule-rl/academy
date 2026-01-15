{- func:
    \x -> x: anonymous func
        id x = x: bound to 'id'
    curry: nesting multiple args -> 1 arg 1 res
    func take '2' args -> 2 funcs ret 1 res
    datatype: data
        type constructor, zero, more data constructors which have zero/args -}
uncurry' :: (a -> b -> c) -> ((a, b) -> c)
curry': f a b = f (a, b)
    add :: (Int, Int) -> Int
        add' = curr' add
    f a b = a + b
        f = \a -> (\b -> a + b)
{- pattern matching: datatypes
    deconstruct/unpack types, product/sum types 2 or more args
        single val: can ignore completely for non-sum constuctor changes nothing
        product: bind vals to names a -> x
            tuple/data constructor: a set of types compounded over each other
        sum: choose which inhabitant to handle/ignore
            |: or, terms in either type, not simultaneously -}
data Blah = Blah
blahFunc :: Blash -> Bool
blahFunc Blah = True

data Identity a = 
    Identity a 
    deriving (Eq, Show)
unpackIdentity :: Identity a -> a
unpackIdentity :: (Identity x) = x
ignoreIdentity :: Identity a -> Bool
ignoreIdentity (Identity _) = True
ignoreIdentity' ::Identity a -> Bool
ignoreIdentity' _ = True

data Product a b = 
    Product a b
    deriving (Eq, Show)
ProductUnpackOnlyA :: Product a b -> a
ProductUnpackOnlyA (Product x _) = x
ProductUnpack :: Product a b -> a
ProductUnpack (Product x y) = (x, y)

data SumOfThree a b c = 
    FirstPossible a
    | SecondPossible b
    | ThirdPossible c
    deriving (Eq, Show)
sumToInt :: SumOfThree a b c -> Integer
sumToInt (FirstPossible _) = 0
sumToInt (SecondPossible _) = 1
sumToInt (ThirdPossible _) = 2

sumToInt :: SumOfThree a b c -> Integer
sumToInt (FirstPossible _) = 0
sumToInt _                 = 1

{- bottom: signaling, eval program laziness
    loop infinitely
    doenst handle all inputs, fail on pattern match -}
f x = f x
dontPassFalseVal :: Bool -> Int
dontPassFalseVal True = 1
    dontPassFalseVal False = error "shouldve written this line or better"

{- high order func: func take funcs as arag
    take more than 1 arg
    composition: application of func applied another func -}
(.) :: (b -> c) -> (a -> b) -> a -> c
    (.) :: (b -> c) -> (a -> b) -> (a -> c)
    (.) :: (b -> c) -> ((a -> b) -> (a -> c))
comp f g x = f (g x)

{- pointfree: w/ arg 
    not pointfree
        zipwith(+): ele addition of 2 lists
        \l -> :flatten one level after mapping d
        map d l: apply d recursively to each ele of l
        >>= \h -> h: conat/join the lists
    pointfree verseion -}
blah x = x
addAndDrop x y = x + 1
reverseMkTuple a b = (b, a)
reverseTuple (a, b) = (b, a)
what d = zipWith (+) (\l -> (map d l) >>= \h -> h)
    blah = id
    addAndDrop = const . (1 +)
    reverseMkTuple = flip (,)
    what = zipWith (+) . (join .) . map

{- recursion
    not recursive
    recursive
        base case as backstop -}
lessOne :: Int -> Int
lessOne n = n - 1
    zero :: Int -> Int
    zero 0 = 0
    zero n = zero (n - 1)
    
{- typeclass inheritance
    instance of superclass num first, then instance of Fractional
    derived instances for Eq, Enum, Ord, Show-}
class Num a => Fractional a where
    (/) :: a -> a -> a
    recip :: a -> a
    fromRational :: Rational -> a
        instance Num numFirst where
            ...
        instance Fractional Nada where
            (Nada x) / (Nada y) = Nada (x / y)
            recip (Nada n) = Nada (recip n)
            fromRational r = Nada (fromRational r)

{- cons: : x : xs 1 : [2,3] == [1, 2, 3]
        : data constructor, prepend val onto the head of another list val 
    cons cell: list datatype, data constructor/product types a [a] 
       nesting of multiple cons cells
            ref itself in second arg 
            (Cons 1 ...), (Cons 2 ...), (Cons 3 Nil) 
                all individual cons in list [1, 2, 3]
    spine: recursive nesting of cons cells
        the structure of collection that isnt the vals -}
1 : [2, 3]  
    [1, 2, 3]
(:) :: a -> [a] -> [a]
    data [] a = [] | a : [a]
        data List a = Nil | Cons a (List a)
    Cons 1 (Cons 2 (Cons 3 Nil))
        1 :
          (2 :
             (3 : --|
                  []))
        _ :
          (_ : 
              (_ :
                  []))

{- fold: high-order func 
        accumulate res, recursive 
            ref to collections of vals ref by a recursive datatype
    catamorphism: arbitrary datatype
        break down structure
    tail call: final res of func
        h (subFunc x y z) -> h
        calls h
    tail recursion: tail calls recursively invocate itself
        not recursive:
            calls h
            foldr gives up control to f before continuing the list
        recursive:
            f calls itself, foldl invokes itself  -}
Data.Bool 
data Bool = False | True
    bool :: a -> a -> Bool -> a
data Maybe a = Nothing | Just a
    maybe :: b -> (a -> b) -> Maybe a -> b
data Either a b = Left a | Right b
    either :: (a -> c) -> (b -> c) -> Either a b -> c
        f x y z = h (subFunc x y z)
            where subFunc x y z = g x y z
        foldr f z []    = z
        foldr f z (x:xs) = f x (foldr f z xs)
            f x y z = f (x - 1) y z
            foldl f z []    = z
            foldl f z (x:xs) = foldl f (f z x) xs
        
{- higher kinded type: kind has func arrow -> 
        type constructor, rather than type constant
    higher kind than *
    not higher -}
Maybe :: * -> *
[] :: * -> *
Either :: * -> * -> *
(->) :: * -> * -> *
    Int :: *
    Char :: *
    String :: *
    [Char] :: *
