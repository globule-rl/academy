module Mo where

sayHello :: String -> IO ()
sayHello x = putStrLn ("Hello, " ++ x ++ "!")

main :: IO ()
main = do
  putStrLn secondGreeting
  where secondGreeting = concat [hello, " ", world]
                         (++) "hello" " world!"

-- GHCi 
Prelude> :l hello.hs
Prelude> sayHello "x"
-- Hello, x!
Prelude> :q

{- REPL read-eval-print loop
    x->head of lambda x*3->body
    continue the module, showing prompt: :m
    no typeclass/Num constraint, Integer impl Num -}
Prelude> let triple x = x * 3
Prelude> :t triple
-- triple :: Integer -> Integer
Prelude> triple 2 
  (triple x = x * 3)2
-- 6
Prelude> :i (+) (-)
-- infixl 6 +, -
Prelude> :t 'a'
-- 'a' :: Char
Prelude> :t "Hello!"
-- "Hello!" :: [Char]
Prelude> triple :: Integer -> Integer
Prelude> triple x = x * 3
-- let triple x = x * 3 :: Integer
Prelude> :t triple
-- triple :: Integer -> Integer

x = 5
y = (1 -)
myResult = y x
-- -4

{- print: side-effect interact with the outside world effect -}
printInc n = print plusTwo
  where plusTwo = n + 2
printInc2 n = let plusTwo = n + 2
              in print plusTwo
   \plusTwo -> lamda
printInc2' n =
  (\plusTwo -> print plusTwo) (n + 2)
c where a = b -- (\a -> c) b

data Bool = False | True

greetIfCool :: String -> IO ()
greetIfCool coolness = 
  if cool coolness
    then putStrLn "cool"
  else
    putStrLn "not cool"
  where cool v = v == "cool"

-- tuple (Integer, String)
let myTup = (1 :: Integer, "b")
fst myTup
-- 1
snd :: (a, b)
-- b
2 + fst (1, 2)
-- 3

-- typeclass/interface, a set of types Num: Integer, Fractional, Real: Double
{-  currying: each func take one arg, ret one res
      nested seccessive for multi func objs -> many funcs, one arg
      outmost layer ret another func that accepts next arg  -}
(Ord a, Num a) => a -> Ordering
(Num a, Num b) => a -> b -> b
-- a -> (b -> b) right associative infix op
Prelude> :t addStuff
addStuff :: Integer -> Integer -> Integer
Prelude> addStuff 5 5
-- 15
-- addStuff 5 -> 10, addStuff 5 5 -> 15
useNumNotWork :: Num a => a -> a -> a
useNumNotWork x y = (x / y) + 1
-- type error, cant deduce fractional from num
useFactionalWorks :: Fractional a => a -> a -> a
useFactionalWorks x y = (x / y) + 1
-- works

{- type constructor arg -> res 
    outmost/first arg/a -> res/nested (b -> (c -> string)) -}
Prelude> :i (->)
-- data (->) a b
    a -> b -> c -> string <=> (a -> (b -> (c -> string)))
funcIgnoresArgs :: a -> a -> a -> string
funcIgnoresArgs x y z = "Blah"
Prelude> :t funcIgnoresArgs
-- funcIgnoresArgs :: a -> a -> a -> string
Prelude> :t funcIgnoresArgs (1 :: Integer)
-- funcIgnoresArgs (1 :: Integer) :: Integer -> Integer -> string
-- undefined: any types, bottom
Prelude> let u = undefined
Prelude> :t funcIgnoresArgs u
-- funcIgnoresArgs undefined :: a -> a -> String
Prelude> :t funcIgnoresArgs u u u 
-- funcIgnoresArgs u u u :: String
Prelude> funcIgnoresArgs u u u
-- "Blah"

-- manual, \b lambda
nonsense :: Bool -> Integer
nonsense False = 31
anonymous :: Integer -> Bool -> Integer
anonymous = \i b -> i + (nonsense b)
anonymousAndManuallyNested :: Interger -> Bool -> Integer
anonymousAndManuallyNested = \i -> \b -> i + (nonsense b)
Prelude> anonymous 10 False
-- 41
Prelude> anonymousAndManuallyNested 10 False
-- 41

-- uncurry tuple
Prelude> let uncurry f (a, b) = f a b
Prelude> :t uncurry
-- uncurry :: (t1 -> t2 -> t) -> ((t1, t2) -> t)
Prelude> (+) 1 2
-- 3
Prelude> uncurry (+) (1, 2)
-- 3

Prelude> 6 / fromIntegral (length [1, 2, 3])
-- 2.0

data Mood = Blah
instance Show Mood where
  show _ = "Blah"

*Main> Blah
-- Blah
Prelude> data Mood = Blah deriving Show
Prelude> Blah
-- Blah

-- instance of typeclass
class Numberish a where
  fromNumber :: Integer -> a
  toNumber :: a -> Integer
newtype Age = 
  Age Integer
  deriving (Eq, Show)
instance Numberish Age where
  fromNumber n = Age n
  toNumber (Age n) = n
newtype Year = 
  Year Integer
  deriving(Eq, Show)
instance Numberish Year where
  fromNumber n = Year n
  toNumber (Year n) = n
sumNumberish :: Numberish a => a -> a -> a
sumNumberish a a' = fromNumber summed
  where integerOfA = toNumber a
        integerOfAPrime = toNumber a'
        summed = integerOfA + integerOfAPrime

Prelude> sumNumberish (Age 10) (Age 10)
-- Age 20
Prelude> :t sumNumberish (Age 10)
-- sumNumberish (Age 10) :: Age -> Age

data Trivial = 
  Trivial'
-- impl eq for Trivial
instance Eq Trivial where
  Trivial' == Trivial' = True
prelude> Trivial' == Trivial'
-- true

{- all warnings -}
Prelude> :set -Wall
Prelude> :l hello.hs

f :: Int -> Bool
f 1 = True
f 2 = True
f _ = False

-- have to have Eq instance to define a
data Identity a = 
  Identity a
instance Eq a => Eq (Identity a) where
  (==) (Identity v)(Identity v') = v == v'

-- right-associative?
data DayOfWeek =
  Mon | Tue | Weds | Thu
  deriving (Ord, Show)
    deriving (Eq, Show)
  instance Ord DayOfWeek where
    compare Weds _ = GT

*Main> Mon > Tue
-- False
*Main> compare Mon Tue
-- LT
*Main> compare Weds Thu
-- -- GT

addOrd :: (Ord a, Num a) => a -> a -> a
addOrd x y =
  if x > 1
  then x + y
  else x
concreteType :: Int -> Int -> Int
concreteType x y =
  if x > 1
  then x + y
  else x
