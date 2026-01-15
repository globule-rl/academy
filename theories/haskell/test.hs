module Addition where
{- context :: String -> SpecWith a -> SpecWith a
    example :: Expectation -> Expectation
    specify :: Example a => String -> a -> SpecWith (Arg a) -}
import Test.Hspec

{- do: nested do, sequence monadic actions
    shouldBe :: (Eq a, Show a) => a -> a -> Expectation
        (==) :: Eq a => a -> a -> Bool
    go: recursion cnt+1 (num-denominator)/d = (cnt, n)
        |: pattern matching/guards -}
main :: IO ()
main = hspec $ do
    describe "Addition" $ do
        it "1 + 1 is greater than 1" $ do
            (1 + 1) > 1 `shouldBe` True
dividedBy :: Integral a => a -> a -> (a, a)
dividedBy num denom = go num denom 0
    where go n  d cnt
        | n < d = (cnt, n)
        | otherwise = go (n - d) d (cnt + 1)
main :: IO ()
main = hspec $ do
    describe "Addition" $ do
        it "15 divided by 3 is 5" $ do
            dividedBy 15 3 `shouldBe` (5, 0)

{- Gen: generate pseudorandom vals
    return :: Monad m => a -> m a 
        return :: a -> Gen a
    elements [1, 2, 2, 2, 2, 3] to increase probability
    frequency: weigh in probability -}
trivialInt :: Gen Int
trivialInt = return 1
genInt :: Gen Int
genInt = elements [1, 2, 3]
genBool :: Gen Bool
genBool = choose (False, True)
genTuple :: (Arbitrary a, Arbitrary b) => Gen (a, b)
genTuple = do
    a <- arbitrary
    b <- arbitrary
    return (a, b)
smaple (genTuple :: Gen ([()], Float))
genEither :: (Arbitrary a, Arbitrary b) => Gen (Either a b)
genEither = do
    a <- arbitrary
    b <- arbitrary
    elements [Left a, Right b]
genMaybe' :: Arbitrary a => Gen (Maybe a)
genMaybe' = do
    a <- arbitrary
    frequency [ (1, return Nothing)
                , (3, return (Just a))]
data Fool = 
    Fulse
  | Frue
  deriving (Eq, Show)

{- qc property test type: x :: Int -}
import Test.QuickCheck
    it "x + 1 is always greater than x" $ do
        property $ \x -> x + 1 > (x :: Int)
prop_addGreater :: Int -> Bool
prop_addGreater x = x + 1 > x
runQc :: IO ()
runQc = quickCheck prop_addGreater

{- idempotence: sorted list, apply the same func wont change it -}
twice f = f . f
fourTimes = twice . twice
f x =
    capitalizeWord x
    == twice capitalizeWord x
    == fourTimes capitalizeWord x
f x =
    sort x
    == twice sort x
    == fourTimes sort x