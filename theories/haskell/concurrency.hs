{- data structure to represent your problem
    affect speed and mem involved in processing your data
    parallelism/concurrency
    defaultmain: apply arg to func, res not shared
        re-perform for each sampling
    whnf: week head normal form 
        eval to the first data constructor
            only eval outer Maybe/find out Nothing/Just
                no counting of eval a val
    nf: normal form, eval all
    !!: report version inline
        foldr: Int -> a accumulator optimization 
            select nth ele of xs, foldr recursion on index k
            return tooLarge if n out of bounds
    !?: left associating infix operator with a precedence of 9
       :: type declaration to improve performance
            Int better than Num a => a infer to int
    inline: func at call sites for performance, ignore thresholds
        inlineable: cross module inlining, allow specialization, performance-critical exports -}
defaultMain :: [Benchmark] -> IO ()
whnf :: (a -> b) -> a -> Benchmarkable
nf :: Control.DeepSeq.NFData b =>
        (a -> b) -> a -> Benchmarkable

module Main where
import Criterion.Main
#ifdef USE_REPORT_PRELUDE
xs !! n | n < 0 = error "Prelude.!!: negative index"
[] !! _         = error "Prelude.!!: index too large"
(x:_) !! 0      = x
(_:xs) !! n     = xs !! (n-1)
#else
    xs !! n
        | n < 0     = negIndex
        | otherwise =
            foldr (\x r k -> case k of 
                                0 -> x
                                _ -> r (k-1)) tooLarge xs n

infixl 9 !?
(!?) :: [a] -> Int -> Maybe a
_       !? n | n < 0 = Nothing
[]      !? _         = Nothing
(x:_)   !? 0         = Just x
(_:xs)  !? n         = xs !? (n-1)
myList :: [Int]
myList = [1..9999]
main :: IO ()
main = defaultMain
    [ bench "index list 9999"
        $ whnf (myList !!) 9998
    , bench "index list maybe index 9999"
        $ whnf (myList !?) 9999 ]
[1,2,3] !? 2
(_:[2,3]) !? 2
  = [2,3] !? (2-1)
[_:[3]] !? 1
    = [3] !? 0
{-# INLINABLE (!?) #-}
(!?) :: [a] -> Int -> Maybe a
xs !? n
  | n < 0       = Nothing
  | otherwise   = 
        foldr (\x r k -> case k of
                    0 -> Just x
                    _ -> r (k-1))(const Nothing) xs n

{- bottom ⊥: in recursion, do not res in val, false
        fail w/ err, fail to terminate
            let x = x in x
    whnf enforcing stopping point
        func val that ret bottom val instead of data constructor/Maybe to stop
    whnf error guarded recursion: 
        data constructor/map interpose between each recursion stop 
            step up to whnf incrementally on demand
        eval only (_ : _) the first cons cell
            -> use nf
        slower than indexing, also constructing a new list
         -}
import Debug.Trace
myList :: [Int]
myList = trace "myList was evald"
        ([1..9999] ++ [undefined])
-- (!?)
main :: IO ()
main = defaultMain
            > criterion1: Prelude.undefined
                (Just undefined) `seq` 1 
                                    == 1
                ((\_ -> Just undefined) 0) `seq` 1
                                            == 1 
myList :: [Int]
myList = [1..9999]
    myList = (undefined : [2..9999])
    myList = (undefined : undefined)
    myList = undefined
main :: IO ()
main = defaultMain
    [ bench "map list 9999" $ nf (map (+1)) myList]
            > time 122.5us

{- profiling:
    see why slow/faster, where spend time
    (_:xs) !! n     = xs !! (n-1)
    cost center
        g -> 91.2% f -> 8.8%
    mem -}
f :: IO ()
f = do
    print ([1..] !! 999999)
    putStrLn "f"
g = IO ()
g = do
    print ([1..]!! 9999999)
    putStrLn "g"
main :: IO ()
main = do
    f
    g
> stack ghc -- -prof -fprof-auto -rtsopts -02 profile.hs
    > total time = 0.22 secs
import Control.Monad
blah :: [Integer]
blah = [1..1000]
main :: IO ()
main = replicateM_ 10000 (print blah)

{- caf constant applicative form
            expr/val/func w/ no free var, held in mem to be shared with other exprs
        no re-eval shared vals 
            but mem-intensive quickly -> find goldenlock zone
        w/o arg: pointfree
    avoid caf: 
        w/ arg: pointful 
        wont stay in heap 
            list/control structure is cheap to construct and thrown away
    > cost center incdInt main
        %time       90%  9.9%
        %alloc      100%  0  total alloc: 1,440,216,712 bytes more
        arg-time    92.6  7.4 4,560,544,472 bytes less -}
incdInts :: [Integer]
incdInts = map (+1) [1..]
main :: IO ()
main = do
    print (incdInts !! 1000)
    print (incdInts !! 9001)
    print (incdInts !! 9901000)
        > total time = 0.28 secs
incdInt :: a -> [Integer]
incdInts _ = map (+1)[1..]
main :: IO ()
main = do
    print (incdInts 0 !! 1000)
    print (incdInts 0 !! 9901000)    
        > total time = 0.93 secs    
incdInt :: [Integer] -> [Integer]
incdInts = map (+1)
        > CAF
            incdInts
            main
                incdInts
incdInts x = map (+1) x
main :: IO ()
main = do
    print (incdInts [1..] !! 1000)
        > CAF
            main
                incdInts

{- map: key: val container
    !: strictness bang unpack
     Tip: cap off branch of a tree
     0: "0" .. 9001: "9001"
     persistent
    set: key ordered
    sequence; append front/cons & back
            fingertree: O(1) to end, O(log n) elsewhere 
        !!: list triple nested 9001 -> 9002 -> 9003 O(n) per access O(9001+9002+9003) 1-based indexing
        flip: access 9001 directly O(logn) O(17) binary tree 
            2^h = n, h=log2(n)           
    vector: mem density, locality -> larger 
        wrapper of array
        boxed, mutable, storable
            boxed: ref any datatype 
            unboxed: raw val w/o ptr, save mem, only Bool, Char, Double, Float, Int, Word, tuples
        index via Int, uniform access time for each ele, ongoing update
        slicing, sub-array, ret a new wrapper, ref to the same arr, no constructing new data
            O(100)
            O(1)
        loop fusion: optimizer, megaloop
            add 4*(+n) to each ele [1+4n .. 10000+4n]
            //: batch op several eles
                replace n with 0
            fmap //
            update V.fromList
            in-place update: mutable vecs
                immutability: data pure persistent
                create new vectors, preserve referential transparency(same val same context)
                preserve equational reasoning (f x = x) 
                    go: tail-recursive
    mutableUpdateIO: the fastest, 4 orders of magnitude -}
data Map k a
    = Bin {-# UNPACK #-} !Size !k a !(Map k a) !(Map k a)
    | Tip
type Size = Int
import Criterion.Main
import qualified Data.Map as M
genList :: Int -> [(String, Int)]
genList n = go n []
    where go 0 xs = ("0", 0) : xs
            go n' xs = go (n' - 1) ((show n', n') : xs)
pairList :: [(String, Int)]
pairList = genList 9001
testMap :: M.Map String Int
testMap = M.fromList pairList
Main :: IO ()
main = defaultMain
    [ bench "look up in list" $
        whnf (lookup "doesntExist") pairList
    , bench "look up in map" $
        whnf (M.lookup "doesntExist") testMap ]

import qualified Data.Set as S
bumpIt (i, v) = (i+1, v+1)
m :: M.Map Int Int
m = M.fromList $ take 10000 stream
    where stream = iterate bumpIt (0, 0)
s :: S.Set Int
s = S.fromList $ take 10000 stream
    where stream = iterate (+1) 0
membersMap :: Int -> Bool
membersMap i = M.member i m
membersSet :: Int -> Bool
membersSet i = S.member i s

data FingerTree a
    = Empty
    | Single a
    | Deep {-# UNPACK #-} !Int !(Digit a)
            (FingerTree (Node a)) !(Digit a)
newtype Elem a = Elem {getElem :: a}            
newtype Seq a = Seq(FingerTree (Elem a))

import qualified Data.Sequence as s
lists :: [[Int]]
lists = replicate 10 [1..100000]
seqs :: [S.seq Int]
seqs = replicate 2 (S.fromList [1..100000])
bench $ nf mconcat lists
bench $ nf mconcat seqs

lists :: [Int]
lists = [1..100000]
seqs :: S.Seq Int
seqs = S.fromList [1..100000]
bench whnf (\xs -> xs !! (xs !! 9001)) lists
bench whnf (flip S.index 9001) seqs

data Vector a = Vector {-# UNPACK #-} !Int
                        {-# UNPACK #-} !Int
                        {-# UNPACK #-} !(Array a)
    deriving ( Typeable )
import qualified Data.Vector as V
slice :: Int -> Int -> [a] -> [a]
slice from to xs = take (to - from + 1) (drop from xs)
l :: [Int]
l = [1..1000]
v :: V.Vector Int
v = V.fromList [1..1000]
bench whnf (head . slice 100 900) l
bench whnf (V.head . V.slice 100 900) v
instance G.Vector Vector a where
    {-# INLINE basicUnsafeSlice #-}
    basicUnsafeSlice j n (Vector i _ arr) = 
        Vector (i+j) n arr

testV n = 
    V.map ((+n) . (+n)
            . (+n) . (+n)) (V.fromList [1..10000])   
testV' :: Int -> V.Vector Int
testV' n =
    V.map (+n) $ V.map (+n) $
        V.map (+n) $ V.map (+n)
        (V.fromList [1..10000])    
bench whnf testV 9998
bench whnf testV' 9998

vec :: V.Vector Int
vec = V.fromList [1..10000]
slow :: Int -> V.Vector Int
slow n = go n vec
    where go 0 v = v
            go n v = go (n-1) (v // [n, 0])
batchList :: Int -> V.Vector Int
batchList n = vec // updates
    where updates = fmap (\n -> (n, 0)) [0..n]
batchVector :: Int -> V.Vector Int
batchVector n = V.unsafeUpdate vec updates
    where updates = fmap (\n -> (n, 0)) (V.fromList [0..n])
bench $whnf slow 9998
bench $whnf batchList 9998   

import Contorl.Monad.Primitive
import Control.Monad.ST
import Criterion.Main
import qualified Data.Vector as V
import qualified Data.Vector.Mutable as MV
import qualified Data.Vector.Generic.Mutable as GM
mutableUpdateIO :: Int -> IO (MV.MVector RealWorld Int)
mutableUpdateIO n = do
    mvec <- GM.new (n+1)
    go n mvec 
    where go 0 v = return v
            go n v = (MV.write v n 0) >> go (n-1) v
mutableUpdateST :: Int -> V.Vector Int 
mutableUpdateST n = do
    mvec <- GM.new (n+1)
    go n mvec
    where go 0 v = V.freeze v
            go n v = (MV.write v n 0) >> go (n-1) v  

{- ST: unfreeze data, mutate it, refreeze, similiar to IO
    code mutating not recorded by optimizer
        s type from mutating a, no val witness
            state monad erasable, encapsulate and melt
    enter closure/lambda perform effect -> side effect mutation
        preserve the order n effects
            each new depend on the prev
    s type: ST enforce at compile-time s not unify w/ outside ST Monad -}
type STRep s a = GHC.Prim.State# s -> (# GHC.Prim.State# s, a #)
newtype ST s a = GHC.ST.ST (GHC.ST.STRep s a)

{- str text
    lazy: read as much text as neccessary
    streaming -}
import Control.Monad
import qualified Data.Text as T
import qualified Data.Text.IO as TIO
import qualified System.IO as SIO
import qualified Data.Text.Lazy as TL
import qualified Data.Text.Lazy.IO as TLIO
dictWords :: IO String
dictWords = SIO.readFile "/usr/share/dict/words"
dictWordsT :: IO T.Text
dictWordsT = TIO.readFile "/usr/share/dict/words"
dictWordsTL :: IO TL.Text
dictWordsTL = TLIO.readFile "/usr/share/dict/words"
main :: IO ()
main = do
    replicateM_ 10 (dictWords >>= print)
    replicateM_ 10 (dictWordsT >>= TIO.putStrLn)

{- Bytestring: vector of Word8 vals
    seq of bytes
        encoding of text: ASCII, UTF-8, UTF-16
        overloadedstrings pragma {-# LANGUAGE OverloadedStrings #-}
            desugar to IsString Text/ByteString
    gzip: include bytes, not a valid text encoding
        cant do   TIO.putStrLn $ TE.decodeUtf8 (s compressed)
                        where s = BL.tostrict
    Char* not for unicode ~, only for ASCII -}
module BS where
import qualified Data.Text.IO as TIO
import qualified Data.Text.Encoding as TE
import qualified Data.Bytestring.Lazy as BL
import qualified Codec.Compression.GZip as GZip
input :: BL.Bytestring
input = "123"
compressed :: BL.Bytestring
compressed = GZip.compress input
main :: IO ()
main = do
    TIO.putStrLn $ TE.decodeUtf8 (s input)

import qualified Data.Text as T
import qualified Data.Text.Encoding as TE
import qualified Data.Bytestring as B
import qualified Data.Bytestring.UTF8 as UTF8
s :: String
s = "\12371\12435\65311"
utf8Print :: B.Bytestring -> IO ()
utf8Print = 
    putStrLn . T.unpack . TE.decodeUtf8
teEncodeWorks :: B.Bytestring
teEncodeWorks = TE.encodeUtf8 (T.pack s)
libWorks :: B.Bytestring
libWorks = UTF8.fromstring s
thisWorks :: IO ()
thisWorks = utf8Print teEncodeWorks
alsoWorks :: IO ()
alsoWorks = utf8Print libWorks

import Data.Char (ord)
ord :: Char -> Int
ord 'A' == 65
ord '\12323' == 12323
let charSeq = map (:[]) ['A'..'']
    mapM_ (utf8Print . B8.pack) charSeq
mapM_ putStrLn (take 3 $ drop 60 charSeq)
    ~

{- queue: prepend/append cheaply
    O(1): front empty, reverse, pop head x, f rest
    q (\(_, q') -> q')
       push i onto queue q, pop, take new queue q' n i+1, recurse with updated -}
data Queue a = 
    Queue { enqueue :: [a]
            , dequeue :: [a]
            } deriving (Eq, Show)
emptyQ :: Queue a
emptyQ = Queue []
push :: a -> Queue a -> Queue a
push x (Queue f r) = Queue f (x : r)
pop :: Queue a -> Maybe (a, Queue a)
pop (Queue [] []) = Nothing
pop (Queue (x : f) r) = Just (x, Queue f r)
pop (Queue [] r) = case reverse r of
                    [] -> Nothing
                    (x : f) -> Just (x, Queue f [])
-- singlelist: 
pop (Queue []) = Nothing
pop (Queue (x : xs)) = Just (x, Queue xs)
alternating :: Int -> (Queue Int, Int) -> (Queue Int, Int)
alternating n (q, i)
    | i < n     = alternating n (maybe q (\(_, q') -> q') (pop (push i q)), i+1)
    | otherwise = (q, i)
bench "two-list" $ nf (snd . alternating 10000) (emptyQ, 0)