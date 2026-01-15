{- monoid: associativity, algebra, folding
    associative binary op, identity ele
    semigrouping: no identity -}
mappend [1..5][] = [1..5]
mappend x mempty = x
class Monoid m where
    mempty :: m
    mappend :: m -> m -> m
    mconcat :: [m] -> m
    mconcat = foldr mappend mempty
foldr (++) [] [[1..3], [4..6]] == [1,2,3,4,5,6]
instance Monoid [a] where
    mempty = []
    mappend = (++)
mappend (Sum 2) (Sum 4) == Sum {getSum = 6}

{- newtype: c union one ptr, a new type/data constructor 
        simple wrap up
    data: extra wrap up
    <>: infix
    cata: reduce recursive data structure to a val via folding
        foldr :: (a -> b -> b) -> b -> [a] -> b
            foldr (+) 0 [1,2,3] -> 6 
    laws: scability -}
data Server = Server String
newtype Server' = Server' String
(Sum 1) <> (Sum 2) <> (Sum 1) 
    == Sum {getSum = 4}
(Sum 1) `mappend` (Sum 1) `mappend` (Sum 1) 
    == Sum {getSum = 3}
mconcat [(Sum 4), (Sum 3), (Sum 2)] 
    == sum {getSum = 9}
getSum $ mappend (Sum 1) (Sum 2) 
    == 3
foldr mappend mempty ([2,4,6] :: [Product Int]) 
    == Product {getProduct = 48}
Last Nothing `mappend` Last (Just 3) 
    == Last {getLast = Just 2}

import Data.Monoid
import Test.QuickCheck
monoidAssoc :: (Eq m, Monoid m) => m -> m -> m -> Bool
monoidAssoc a b c = (a <> (b <> c)) == ((a <> b) <> c)
type S = String
type B = Bool
QuickCheck (monoidAssoc :: S -> S -> S -> B)

mappend mempty x = x
mappend x mempty = x
mappend mempty x = Fools
mappend x mempty = Fools
{- 0 + x == x
    0 * x == 0
    failed test, Twoo, always returns  Fools
        invalid, cant have false as identity -}
import Control.Monad
import Data.Monoid
import Test.QuickCheck
data Bull =
    Fools
  | Twoo
  deriving (Eq, Show)
instance Arbitrary Bull where
    arbitrary = 
        frequency [ (1, return Fools)
                , (1, return Twoo)]
instance Monoid Bull where
    mempty = Fools
    mappend _ _ = Fools
type BullMappend = Bull -> Bull -> Bull -> Bool
main :: IO ()
main = do
    quickCheck (monoidAssoc :: BullMappend)
    quickCheck (monoidLeftIdentity :: Bull -> Bool)
    quickCheck (monoidRightIdentit7 :: Bull -> Bool)              

class Semigroup a where
    (<>) :: a -> a -> a
(a <> b) <> c = a <> (b <> c)
class Semigroup a => Monoid a where 
    ...
import Data.List.NonEmpty as N
import Data.Semigroup as S
{- :|: infix, always have at least one val
    prefix: prefix -}
data NonEmpty a = a :| [a]
    deriving (Eq, Ord, Show)
newtype NonEmpty a = 
    NonEmpty (a, [a])
    deriving(Eq, Ord, Show)
data P = 
    Prefix Int String
data Q = 
    Int :!!:  String
let xs = 1 :| [2, 3]
let ys = 4 :| [5, 6]
xs <> ys == 1 :| [2,3,4,5,6]
N.head xs == 1
N.length (xs <> ys) == 6

{- IO () 
    state token erased in compile time, zero mem
    IO: turn off optimization like reorder op. delay eval, sharing, inline, chaos
        nested lambdas -> sequenced actions 
        only a means of getting val, not val shared
    getCurTime: get old/first time if without IO 
        no sequence when run main another time
    whnf: get avg eval time, not just the first/shared
        io: no need to resort to add an arg, execute over and over
            main in it  -}
import GHC.Prim
import GHC.Types
newtype State s a
    = State {runState :: s -> (a, s)}
newtype IO a 
    = IO (State# RealWorld
        -> (# State# RealWorld, a #))
getCurTime :: IO UTCTime
whnf :: (a -> b) -> a -> Benchmarkable
nf :: NFData b => (a -> b) -> a -> Benchmarkable
whnfIO :: IO a -> Benchmarkable
nfIO :: NFData a => IO a -> Benchmarkable

{- IO: not disable sharing in running IO action/show only once
    inner outer trace  -}
import Debug.Trace
blah :: IO String
blah = return "blah"
blah' = trace "outer trace" blah
woot :: IO String
woot = return (trace "inner trace" "woot")
main :: IO ()
main = do
    b <- blah'
    putStrLn b
    putStrLn b
    w <- woot
    putStrLn w
    putStrLn w

{- IO [Int]: diff vals when true
    but ret same res: list of rand nums
        referential transparency preserved
            same inp -> same res, replace val, program same behavior --}
module IORefTrans where
import Control.Monad (replicated)
import System.Random (randomRIO)
gimmeShelter :: Bool -> IO [Int]
gimmeShelter True = replicatedM 6 (randomRIO (0, 10))
gimmeShelter False = pure [0]
    gimmeShelter True 
        == [1, 8, 7, 9, 10, 4]
    gimmeShelter False
        == [0]

{- IO funtor:
        fmap: construct same effect, transform a->b action
        <*>: func & val arg effect, apply func to val
        join: merge nested IO effects
        fmap produce a new IO action/transform res to IO
    IO applicative: (++)
        lift (+) :: Int -> Int -> Int over two randomIO actions, sequencing their effects 
            apply func to yield IO Int (sum of two random ints)
    IO monad
        IO a: a recipe for making a recipe that produces a
        outer IO effects influence what recipe get inner
     -}
fmap :: (a -> b) -> IO a -> IO b
<*> :: IO (a -> b) -> IO a -> IO b
join :: IO (IO a) -> IO a
fmap (+1) (randomIO :: IO Int)
    (++) <$> getLine <*> getLine
            hello julie -> "hellojulie"
    (+) <$> (randomIO :: IO Int) <*> (randomIO :: IO Int)
        let embedInIO = return :: a -> IO a
        join $ embedInIO (print "It prints with join")
        join $ embedInIO (embedInIO 1) == 1

module NestedIO where
import Data.Time.Calendar
import Data.Time.Clock
import System.Random
huehue :: IO (Either (IO Int) (IO ()))
huehue = do
    t <- getCurTime
    let (_, _, dayOfMonth) = 
            toGregorian (utctDay t)
    case even dayOfMonth of
        True -> 
            return $ Left randomIO
        False ->
            return $ Right (putStrLn "false return right")
blah <- huehue
either (>>= print) id blah
    == -7077123123456546576878

{- MVar: hold val
    IO (MVar a) produce as much as you want
        mv/put mv'/take mv'/take ERROR
            take put take put 
        two MVar w/ diff lifetimes, not a constant ref
    fix: pass stable ref as arg -}
module Whathappens where
import Control.Concurrent
myData :: IO (MVar Int)
myData = unsafePerformIO newEmptyMVar
main :: IO ()
main = do
    mv <- myData
    putMVar mv 0
    mv' <- myData
    zero <- takeMVar mv'
    print zero
        main :: IO ()
        main = do
            mv <- newEmptyMVar
            putMVar mv (0 :: Int)
            zero <- takeMVar mv
            print zero