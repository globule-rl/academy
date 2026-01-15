{- IO a, a unit ()
    do: syntactic sugar, allow for sequencing actions
        sequence monadic actions
    <-: bind
    return :: Monad m => a -> m a
        ret inside a monad IO
    -}
main = do x1 <- getLine
          x2 <- getLine
          return (x1 ++ x2)
main :: IO ()
main = do c  <- getChar
          c' <- getChar
          return (c == c')
                if c == c'
                    then putStrLn "True"
                    else return ()

{- maybe isJust: determine if every char has been discovered or no
        all isJust [Just 'd', Nothing, Just 'g']
            Foldable t => (a -> Bool) -> t a Bool
                fold but dont necessarily contain val/more than one val
        all :: (a -> Bool) -> Maybe a -> Bool
            true/false, does it return true for all of them
    interperse: put space between chars -}
Module Main where
    import Control.Monad (forever)
    import Data.Char (toLower)
    import Data.Maybe (isJust)
    import Data.List (intersperse)
    import System.Exit (exitSuccess)
    import System.Random (randomRIO)

{- [String]: new type String WordList, type synonym to declare 
    lines: split String, ret a list of strs each representing a line
        lines "aaa\bbb" -> ["aaa", "bbb"] 
            length $ lines "aaa\bbb"
        "aaa bbb" -> ["aaa bbb"] 
    deriving: instance  -}
type WordList = [String]
allWords :: IO WordList
allWords = do
    dict <- readFile "data/dict.txt"
    return (lines dict)
        newtype WordList =
            WordList [String]
            deriving (Eq, Show)
        allWords :: IO WordList
        allWords = do
            dict <- readFile "data/dict.txt"
            return $ WordList (line dict)

minWordLength :: Int
minWordLength = 5
maxWordLength :: Int
maxWordLength = 9
{- filter by the length criteria to shorter list
    !!: val at index 0-4
    [1..5] !! 4 == 5
    randomWord': bind gameWords list to randomWord func
        gameWords val can be used as arg to func -}
gameWords :: IO WordList
gameWords = do
    aw <- allWords
    return (filter gameLength aw)
    where gameLength w = 
            let l = length (w :: String)
            in l > minWordLength && l < maxWordLength
randomWord :: WordList -> IO String
randomWord wl = do
    randomIndex <- randomRIO(0, (length wl)-1)
    return $ wl !! randomIndex
        gameWords :: IO WordList
        gameWords = do
            (WordList aw) <- allWords
            return $ WordList (filter gameLength aw)
            where gameLength w =
                let l = length (w :: String)
                in l > minWordLength && l < maxWordLength
        randomWord :: WordList -> IO String
        randomWord (WordList wl) = do
            randomIndex <- randomRIO(0, (length wl)-1)
            return $ wl !! randomIndex
randomWord' :: IO String
randomWord' = gameWords >>= randomWord
        
{- puzzle: Puzzle word to guess, discovered, guessed
        type signature
    freshPuzzle: maybe fmap const, Nothing
    charInWord: elem 3 [1..5] == True
        if elem guess pz, then True else False
    alreadyGuessed: (Puzzle _ _ guessed)
    renderPuzzleChar: pattern matching, _ = '_', m = case m of Nothing -> '_' Just c -> c
        for each char, if charInword, show char, else show _
        >let n = Nothing 
        >fmap rendenPuzzleChar [n, Just 'h', n, Just 'e', n] 
            > _h_e
    fillInChar: insert guess/curGuess into wordChar/pz/All isJust, else Nothing/guessChar/old guessed correct one/guessed
        replace with newFilledInSoFar
    zipWith: f :: (a -> b -> c) -> [a] -> [b] -> [c], apply f to [a] [b] -> [c]
        zipWith (+) [1,2,3] [10, 20, 30] == [11, 22, 33]
        zipper guess: op
    if not curGuess == wordChar, curGuess == guess, curGuess as param, guess as arg
        guessChar: old Nothing/Just correct  -}
data Puzzle = Puzzle String [Maybe Char] [char]
freshPuzzle :: String -> Puzzle
freshPuzzle pz = Puzzle pz (map (const Nothing) pz) []
charInWord :: Puzzle -> Char -> Bool 
charInWord (Puzzle pz _ _) guess = guess `elem` pz
alreadyGuessed :: Puzzle -> Char -> Bool
alreadyGuessed = (Puzzle _ _ guessed) chance = chance `elem` guessed
renderPuzzleChar :: Maybe Char -> Char
renderPuzzleChar (Just guess) = guess
                Nothing = '_'
instance Show Puzzle where 
    show (Puzzle _ discovered guessed) = 
        (interperse ' ' $ fmap renderPuzzleChar discovered) 
        ++ " Guessed so far " ++ guessed
fillInChar :: Puzzle -> Char -> Puzzle
fillInChar (Puzzle pz filledInSoFar guessed) guess = 
    Puzzle pz newFilledInSoFar (guess : guessed)
        where zipper curGuess wordChar guessChar = 
            if wordChar == curGuess
            then Just wordChar
            else guessChar
            newFilledInSoFar = 
                zipWith (zipper guess) pz filledInSoFar
handleGuess :: Puzzle -> Char -> IO Puzzle
handleGuess puzzle guess = do
    putStrLn $ "Your guess was " ++ [guess]
    case (charInWord puzzle guess, alreadyGuessed puzzle guess) of
        (_, True) -> do
            putStrLn "You already guessed"
            return puzzle
        (True, _) -> do
            putStrLn "char in word, fill in"
            return (fillInChar puzzle guess)
        (False, _) -> do
            putStrLn "not in word"
            return (fillInChar puzzle guess)

gameOver :: Puzzle -> IO ()
gameOver (Puzzle wordToGuess _ guessed) = 
    if (length guessed) > 7 then
        do putStrLn $ "You lose, the word was " ++ wordToGuess
        exitSuccess
    else return ()
gameWin :: Puzzle -> IO ()
gameWin (Puzzle _ filledInSoFar _) = 
    if all isJust filledInSoFar then
        do putStrLn "You win"
            exitSuccess
    else return ()

runGame :: Puzzle -> IO ()
runGame puzzle = forever $ do
    gameOver puzzle
    gameWin puzzle
    putStrLn $ "current puzzle " ++ show puzzle
    putStrLn "Guess letter"
    guess <- getLine
    case guess of 
        [c] -> handleGuess puzzle c >>= runGame
            -> putStrLn "must be single char"
main :: IO ()
main = do 
    word <- randomWord'
    let puzzle = freshPuzzle(fmap toLower word)
    runGame puzzle
