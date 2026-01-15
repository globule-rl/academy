{- applicative apply outer f to get 
        g (a->b) => g a -> g b
    runMaybeT: m (Maybe a), wrap m on val a
    pure(pure x): wrap x in innermost g, then f, yield f (g x) 
    f $ x: f x, right-associativity, lowest precedence 
    f: f (g (a->b)), x holds f (g a)
    <*>: extract func from left applicative, apply to val in right
    <$>: fmap, map pure func over applicative f
    <*> <$> f: map <*> over f, yield f (g (<*>)), then apply/<*> to x, f (g b)  
    <-: ref, get the val from structure 
    return: re-embed in monad
    >>=: bind, chain, seq, apply func to sucessful res of first, yield a, feed chaining snd b, 
            if fst fails, skip snd, fail overall   -}
:: Maybe (a -> b) -> Maybe a -> Maybe b
newtype MaybeT m a = 
    MaybeT { runMaybeT :: m (Maybe a) }
instance (Functor f, Functor g) =>
    Functor (Compose f g) where
        fmap f (Compose fga) = Compose $ (fmap . fmap) f fga
instance (Functor m) => 
    Functor (MaybeT m) where
        fmap f (MaybeT ma) = MaybeT $ (fmap . fmap) f ma
innerMost :: [Maybe (Identity (a -> b))] -> [Maybe (Identity a -> Identity b)]
innerMost = (fmap . fmap) (<*>) 
second' :: [Maybe (Identity a -> Identity b)] -> [Maybe (Identity a) -> Maybe (Identity b)]               
second' = fmap (<*>)
final' :: [Maybe (Identity a) -> Maybe(Identity b)] -> [Maybe (Identity a)] -> [Maybe (Identity b)]
final' = (<*>)
typeFuncApply :: [Maybe (Identity (a -> b))] -> [Maybe (Identity a)] -> [Maybe (Identity b)]
typeFuncApply f x = final' (second' (innerMost f)) x
instance (Applicative f, Applicative g) =>
    Applicative (Compose f g) where
        pure x = Compose (pure (pure x))
        Compose f <*> Compose x = Compose ((<*>) <$> f <*> x)        
instance (Applicative m) =>
    Applicative (MaybeT m) where
        pure x = MaybeT (pure (pure x))
        (MaybeT fab) <*> (MaybeT mma) = MaybeT $ (<*>) <$> fab <*> mma

MaybeT m a -> MaybeT m b
newtype MaybeT m a = MaybeT { runMaybeT :: m (Maybe a) }
ma :: m (Maybe a)
v :: Maybe a
y :: a
f :: a -> MaybeT m b
runMaybeT (f y) :: m b -> m (Maybe b)
instance (Monad m) => Monad (MaybeT m) where
    return = pure
    (>>=) :: MaybeT m a -> (a -> MaybeT m b) -> MaybeT m b
    (MaybeT ma) >>= f = MaybeT $ do
                    v <- ma
                    case v of
                        Nothing -> return Nothing
                        Just y -> runMaybeT (f y)
newtype EitherT e m a = EitherT {runEitherT :: m (Either e a)}                        
instance (Monad m) => Monad (EitherT e m) where
    return = pure
    (>>=) :: EitherT e m a -> (a -> EitherT e m b) -> EitherT e m b
    (EitherT mea) >>= f = EitherT $ do
        v <- mea
        case v of
            Left e -> return (Left e)
            Right y -> runEitherT (f y)
            
{- lift from: pure IO func ExceptT, ReadT, StateT 
    -> to ActionT
        ExceptT: trans of Either, ReaderT StateT stacked up
            error/left of ExceptT ret as Text
            right ret IO
    fmap: apply func to each ele in f, list, maybe, preverve structure
        f => (a->b) -> f a -> f b
    fmap Right: wrap as success in ExceptT
    runAM: unwrap to run the stack
    lift to: putStrLn IO () --> get/GET ActionM
        terminal: hello
        html: localhost:3000/beam
        html $: send html res w/ param
    \m ->: outer lamda, get monadic action to lift
    lifting: embed an expr into a larger context/add structure that doesnt do anything -}
type Maybe = MaybeT Identity a
type Either e a = EitherT e Identity a
fmap :: Functor f => (a->b) -> f a -> f b
liftA :: Applicative f => (a->b) -> f a -> f b
liftM :: Monad m => (a->r) -> m a -> m r
class MonadTrans t where
    lift :: (Monad m) => m a -> t m a
newtype ScottyT e m a = 
    ScottyT { runS :: State (ScottyState e m) a}
        deriving ( Functor, Applicative, Monad)
newtype ActionT e m a =
    ActionT { runAM :: ExceptT (ActionError e) 
                        (ReaderT ActionEnv 
                            (StateT ScottyResponse m)) 
                                a}
        deriving ( Functor, Applicative )
type ScottyM = ScottyT Text IO
type ActionM = ActionT Text IO

import Web.Scotty
import Web.Scotty.Internal.Types (ActionT(..))
import Control.Monad.Trans.Except
import Control.Monad.Trans.Reader
import Control.Monad.Trans.State.Lazy hiding (get)
import Data.Monoid (mconcat)
main = scotty 3000 $ do
    get "/:word" $ do
        beam <- param "word"
        (lift :: IO a -> ActionM a) (putStrLn "hello")
        html $ mconcat ["<h1>Scotty, ", beam, " me up </h1"]
stack build scotty
stack ghci
    :l scotty.hs
        == hello
get :: RoutePattern -> ActionM () -> ScottyM ()
(t IO a) --> (ActionM a)
lift :: (Monad m, MonadTrans t) => m a -> t m a
lift :: (MonadTrans t) => IO a -> t IO a
lift :: IO a -> ActionM a
lift :: IO () -> ActionoM ()
type ActionM = ActionT Data.Text.Internal.Lazy.Text IO
instance MonadTrans (ExceptT e) where
    lift = ExceptT . liftM Right
instance MonadTrans (ReaderT r) where
    lift = liftReaderT
liftReaderT :: m a -> ReaderT r m a
liftReaderT m = ReaderT (const m)
    . (\m -> ReaderT (const m))
    ReaderT . const
instance of MonadTrans (StateT s) where
    lift m = StateT $ \s -> do
        a <- m
        return (a, s)     
(ActionT 
    . (ExceptT . fmap Right) 
    . ReaderT . const
    . \m -> StateT (\s -> do
                    a <- m return (a, s))
    ) (putStrLn "hello")

instance of MonadTrans IdentityT where
    lift = IdentityT
instance of MonadTrans MaybeT where
    lift = MaybeT . liftM Just
lift  :: (Monad m) => m a -> t m a
    maybeT :: m (Maybe a) -> MaybeT m a
    (liftM Just) :: Monad m => m a -> m (Maybe a)
(MaybeT . liftM Just) :: Monad m => m a -> MaybeT m a
v :: Monad m => m a
liftM Just :: Monad m => m a -> m (Maybe a)
liftM Just v :: m (Maybe a)
MaybeT (liftM Just v) :: MaybeT m a

{- monadIO: lift all structure
    MaybeT . liftIO :: IO a -> MaybeT m (IO a) 
    liftIO :: MaybeT m a
    lift . liftIO: wrap m a as MaybeT (fmap Just a) -}
liftIO :: IO a -> ExceptT e IO a
liftIO :: IO a -> ReaderT r IO a
liftIO :: IO a -> StateT s IO a
liftIO :: IO a -> ExceptT e (StateT s (ReaderT r IO)) a
import Control.Monad.IO.Class
    liftIO :: IO a -> m a
liftIO . return = return
liftIO (m >>= f) = liftIO m >>= (liftIO . f)
liftIO (putStrLn "hello")
instance (MonadIO m) => MonadIO (IdentityT m) where
    liftIO = IdentityT . liftIO
instance (MonadIO m) => MonadIO (EitherT e m) where
    liftIO = lift . liftIO
instance (MonadIO m) => MonadIO (MaybeT m) where
    liftIO = lift . liftIO
instance (MonadIO m) => MonadIO (ReaderT r m) where
    liftIO = lift . liftIO
instance (MonadIO m) => MonadIO (StateT s m) where
    liftIO = lift . liftIO

{- runMaybeT: define big context, unwrap MaybeT transformer,
            extract inner Maybe a in base monad m m (Maybe a)
        hook: user func stored in state to customize
            maybe: hook might be missing
                case hset of 
                    Nothing  ->
                    Just hset -> 
        hset: look up a hookSet inside cur state
        rfolder: run a recentHook stored in the hookSet
            yield IO Maybe FilePath 
        hoist: -> (m a -> n a) -> t m a -> t n a
            lift pure view/IO-ret hookSet -> maybeT mainCoroutine
                wrap FilePath in Just
        r: lift to IO, unwrap via runMaybeT
    query eles safely via MaybeT -> JSRef
        set up async click event in cur doc
            create ref -> JSRef, attach listener to obj/del for management -}
recentFolderHook :: MainCoroutine (Maybe FilePath)
recentFolderHook = do
    xstate <- get
        (r :: Maybe FilePath) <- runMaybeT $ do
            hset <- hoist (view hookSet xstate)
            rfolder <- hoist (H.recentFolderHook hset)
            liftIO rfolder
            return r
addT :: FilePath -> FilePath -> IO (Maybe Integer)
addT f1 f2 = runMaybeT $ do
    s1 <- sizeT f1
    s2 <- sizeT f2
    return (s1 + s2)
main = do
    clickbarref <- asyncCallback1 AlwaysRetain clickbar
    clickbazref <- asyncCallback1 AlwaysRetain clickbaz
    r <- runMaybeT $ do
        doc <- MaybeT curDoc
        bar <- lift . toJSRef
                =<< MaybeT (documentQuerySelector doc (".bar" :: JSString))
        baz <- lift . toJSRef
                =<< MaybeT (documentQuerySelector doc (".baz" :: JSString))
        lift $ do
            ref <- newObj
            del <- delegator ref
            addEvent bar "click" clickbarref
            addEvent baz "click" clickbazref
    case r of
        Nothing -> print "wrong"
        Just _ -> print "well done"

{- temp extend additional structure to avoid boilerplate
    safe param': ret Maybe a on parse failure via rescue
        i: parse "num" query i as Maybe integer/print
        beam: respond w/ html using "word"
    big bind of MaybeT -> Maybe Reco
        could fail w/o needed param
        runMaybeT: unpack MaybeT into ActionM/Maybe Reco
        liftIO: only once instead of two lift for MaybeT n ActionM -}
param' :: Parsable a => Text -> ActionM (Maybe a)
param' k = rescue (Just <$> param k)
                    (const (return Nothing))
main = scotty 3000 $ do
    ...
    beam' <- param' "word"
    let beam = fromMaybe "" beam'
    i <- param' "num"
    liftIO $ print (i :: Maybe Integer)
    html $ mconcat ["<h1>Scotty, ", beam, " me up</h1>"]

type Reco = (Integer, Integer, Interger, Integer)
reco <- runMaybeT $ do
    a <- param' "1"
    liftIO $ print a
    b <- param' "2"
    c <- param' "3"
    d <- param' "4"
    (lift . lift) $ print b
    return ((a, b, c, d) :: Reco)
    liftIO $ print reco