Module Morse
    ( Morse
    , charToMorse
    , morseTochar
    , strToMorse
    , letterToMorse
    , morseToLetter
    ) where
{- map: balanced binary tree, node key/index/Ord:val
    search more efficient than list: ordered left right
    M: Data.Map letterToMorse -> definition
    3 maybes: not every char has a Morse -}
import qualified Data.Map as M
type Morse = Str
letterToMorse :: (M.Map Char Morse)
letterToMorse = M.fromList [
    ('a', ".-")
    ,('b', "-...")
    ,('c', "-.-.")
    ,('0', "-----")
]
morseToLetter :: M.Map Morse Char
morseToLetter = M.foldWithKey (flip M.insert) M.empty letterToMorse
charToMorse :: Char -> Maybe Morse
charToMorse c = M.lookup c letterToMorse
strToMorse :: String -> Maybe [Morse]
strToMorse s = sequence $ fmap charToMorse s
morseTochar :: Morse -> Maybe char
morseToChar m = M.lookup m morseToLetter
