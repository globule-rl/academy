module Main where

{- qualified: prefixed names, alias as M -}
import qualified Data.map as M
import Morse
import Test.QuickCheck

allowedChars :: [char]
allowedChars = M.keys letterToMorse
allowedMorse :: [Morse]
allowedMorse = M.elems letterToMorse
charGen :: Gen Char
charGen = elements allowedChars
morseGen :: Gen Morse
morseGen = elements allowedMorse

{- charToMorse c bound to morseToChar, compare to Just c
    ret Maybe Morse >>= ret Maybe Char, yielding Maybe Char c
    verify bidirectional, c maps to Morse and back to itself -}
prop_thereAndBackAgain :: Property
prop_thereAndBackAgain =
    forAll charGen
    (\c -> ((charToMorse c) >>= morseToChar) == Just c)
main :: IO ()
main = quickCheck prop_thereAndBackAgain
