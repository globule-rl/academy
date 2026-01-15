val + context
2 -> (+3)2 -> 5
data maybe a = Nothing | Just x (x :: a)
    fmap knows the context

typeclass: interface
    (==) :: (Eq a) => a -> a -> Bool
    class constraint => 
        the same type of two vals must be member of Eq class
functors: apply func to a wrapped val
    a data type that impl Functor typeclass
    Maybe impl Funtor/Applicative/Monad
    fmap
        fmap (+3) (Just 2)
            Just 5
        fmap (+3) Nothing 
            Nothing
        Fmap :: (a->b)->fa->fb
            (a->b): (+3)
            fa: Just 2
            fb: Just 5
        instance Functor Maybe where
            fmap func (Just val) = Just (func val)
            fmap func Nothing = Nothing
        fmap (getPostTitle) (findPost 1)
            post = Post.find_by_id(1), if post return post.title, else return nil, end
            if returns post, ret title, if not ret nothing
    <$>: infix version of fmap
        getPostTitle <$> (findPost 1)
    lists
        instance Functor [] where
            fmap = map
        fmap (+3) [2, 4, 6] == [5, 7, 9]
    functions
        instance Functor ((->) r) where
                fmap f g = f . g
        import Control.Applicative
        let foo = fmap (+3) (+1)
        foo 10 == 14
applicatives: apply a wrapped func to wrapped val
        define <*>
    Just (+3) <*> Just 2 == Just 5
    lists
        [(*2), (+3)] <*> [1, 2, 3] == [2, 4, 6, 4, 5, 6]
    funcs that take two args to two wrapped vals
        (+) <$> (Just 5) == Just (+5)
        Just (+5) <*> (Just 4) == Just 8
    liftA2 (*) (Just 5) (Just 3) == Just 15
        (*) <$> Just 5 <*> Just 3 == Just 15
monad: apply a func that returns a wrapped val to a wrapped val
            statically typed filter
        >>== bind
            bind unwrap the val
    half x = if even x
                then Just (x `div` 2)
                else Nothing
        Just 3 >>= half == Nothing
        Just 4 >>= half == Just 2
    class Monad m where
        (>>=) :: m a -> (a -> m b) -> m b
            take a monad/Just 3, and func that rets monad/half, returns a monad/mb
    instance Monad Maybe where
        Nothing >>= func = Nothing
        Just Val >>= func = func val
    chain calls
        Just 20 >>= half >>= half >>= half == Nothing
    IO monad: return wrapped val
        getLine :: IO String
            no arg, get user inp
        readFile :: FilePath -> IO String
            take str/filename, ret file content
        putStrLn :: String -> IO ()
            take str, print it
        getLine >>= readFile >>= putStrLn
    do:
        foo = do
            filename <- getLine
            contents <- readFile filename
            putStrLn contents
        
        