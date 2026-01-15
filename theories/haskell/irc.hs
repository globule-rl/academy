{- irc: internet relay chat
    system: base
    network
        config
        top-level program
            h: wrapped as a Handle, r/w, disable buffering
            write nickname, username, a channel to join
         -}
import System.IO
import qualified Network.Socket as N
import Control.Exception
import Control.Monad.Trans.IO.Class
import Control.Monad.Trans.Reader
import Data.List
import System.Exit
import Data.Time

myServer = "irc.libera.chat"::String
myPort = 6667::N.PortNumber
myChan = "#tutbot-testing"::String
myNick = "tutbot"::String

{- old mean: set no buffering, give commands
    new mean: Control.Exception.bracket: delimit conn, shutdown main loop phases
        bracket takes 3 args: func conn, disconn, loop run in between
            control structure, forever
    runReaderT: st take initial bot state, eval run loop in Net monad -}
main::IO()
main = do
        h <- connectTo myServer myPort
        t <- hGetContents h
        hSetBuffering stdout NoBuffering
        print t
        write h "NICK" myNick
        write h "USER" (myNick ++ " 0 * irc chatbot")
        write h "JOIN" myChan
        listen h
    main = bracket connect disconnect loop
        where
            disconnect = hClose . botSocket
            loop st    = runReaderT run st

{- reader monad: thread a state val
    wrapper over IO, carry immutable state
        define Bot type: struct storing socket, starttime
        ReaderT: type constructor/func, take 2 types as args, build Net monad type
        Control.Monad,Trans.Reader: monad transformer, layer over IO
             IO, global read-only val 
        connect to server, return initial bot state
        notify: control structure -}
data Bot = Bot {botSocket :: Handle, startTime :: UTCTime}
type Net = ReaderT Bot IO
connect :: IO Bot
connect = notify $ do
    h <- getCurrentTime
    h <- connectTo myServer myPort
    return (Bot h t)
  where
    notify a = bracket_
        (putStrLn ("Conn to " ++ myServer ++ " ...") >> hFlush stdout)
        (putStrLn "done")
        a

{- connect to server, return handle
            connect the socket at addr -}
connectTo::N.HostName -> N.PortNumber -> IO Handle
connectTo host port = do
    addr: _ <- N.getAddrInfo Nothing (Just host) (Just(show port))
    sock <- N.socket (N.addrFamily addr) (N.addrSocketType addr) (N.addrProtocol addr)
    N.connect sock (N.addrAddress addr)
    N.socketToHandle sock ReadWriteMode

{- Net monad, join a channel, process commands -}
run :: Net()
run = do
    write "NICK" myNick
    write "USER" (myNick ++ " 0 * irc chatbot")
    write "JOIN" myChan
    listen

{- old write: message to a handle
            3 args: handle/socket, 2 strs irc protocol, args
            concat str
            write/send msg on the wire, show sent msg on the cmd line
        output> NICK tutbot
            USER tutbot 0 * :irc chatbot
            JOIN #tutbot-testing 
    new write: type Net monad, ask for socket when needed instead of threading
        bot connected to server, thus socket initialised 
        liftIO :: IO a -> Net a 
            from Control.Monad.IO,Class -}
write::Handle -> String -> String -> IO()
write h cmd args = do
    let msg = cmd ++ " " ++ args ++ "\r\n"
    hPutStr h msg
    putStr ("> " ++ msg)
        write :: String -> String -> Net ()
        write cmd args = do
            h <- asks botSocket
            let msg = cmd ++ " " ++ args ++ "\r\n"
            liftIO $ hPutStr h msg
            liftIO $ putStr("> " ++ msg)
        command line

{- old listen: handle io
    new listen: net (), h <- ask botSocket, liftIO $/()
    process each line from the server
    forever do: inifinite loop reading lines of text from network, print
        lazy evaluaion
        higher order func
            loop control forever a = a>>forever a
    IRC protocol generate line>
        :dons!i=dons@my.net PRIVMSG #tutbot-testing :hello
        :dons!i=dons@my.net PRIVMSG #tutbot-testing :!id foo
    clean: substr of "key:val" 
        drop first char, prefix until colon, colon -> val
        .: func chain, (f . g) x = f (g x) 
        drop 6: drop first 6 chars "PING :"
        (): one arg to func write, a single val str -}
listen :: Handle -> IO()
    listen :: Net ()
listen h = forever $ do
    line <- hGetline h
    putStrLn line
        h <- asks botSocket
        line <- liftIO $ hGetline h
        liftIO (putStrLn line)
    let s = init line
    if isPing s then pong s else eval h (clean s)
   where
    forever :: IO() -> IO()
        forever :: Net () -> Net()
    forever a = do a; forever a

    clean::String -> String
    clean = drop 1 . dropWhile (/= ':') . drop 1

    isPing::String -> Bool
    isPing x = "PING:" `isPrefixOf` x

    pong::String -> IO()
    pong x = write h "PONG" (':': drop 6 x)

{- eval: dispatch a cmd
    if !quit, exit, if begin w/ '!id', echo arg back, identity func id :: a -> a, id x = x) 
        if no match, do nothing
    new eval: return: ignore everything else -}  
eval :: Handle -> String -> IO ()
eval h "!quit"
eval h x | "!id" `isPrefixOf`
eval _ _
    eval :: String -> Net ()
    eval "!uptime" = uptime >>= privmsg
    eval "!quit" = write "quit" ":Exiting" >> liftIO exitSuccess
    eval x | "!id" `isPrefixOf` x = privmsg (drop 4 x)
    eval _ = return () 

{- send privmsg to cur chan/server -}
privmsg :: Handle -> String -> IO()
privmsg h s = write h "primsg" (chan ++ " :" ++ s)
    privmsg :: String -> Net ()
    privmsg msg = write "primsg" (myChan ++ " :" + msg)

{- get cur uptime 
    t: total num of secs, recursively divide t by each unit
        for each (secs, unit) in the list of time units
            (secs, unit): (86400, "d")
            (n, t'): divMod Integral -> (quotient, remainder) 
                keep breaking down the remainder using smaller units
                93784 `divMod` 86400 == (1, 7384)
                7384 `divMod` 3699 == (2, 184)
            [], in: construct a list
                (n, unit): (1, "d")
                metrics t': remaining time t' with the rest of the units metrics
    filter: remove entries where cnt/fst = 0
        filter(...[(1,"d"), (0,"h"), (3,"m"), (4,"s")] -> [(1,"d"),(3,"m"),(4,"s")])
    floor diff = 93784 diffs = [(1,"d"), (2,"h"), (3,"m"), (4,"s")]
    16:03 dons> !uptime
    16:03 tutbot> 5d 1m 51s -}
uptime :: Net String
uptime = do
    now <- liftIO getCurrentTime
    zero <- asks startTime
    return (pretty (diffUTCTime now zero))
pretty :: NominalDiffTime -> String
pretty diff = 
    unwords
        . map  (\(t, unit) -> show t ++ unit)
        $ if null diffs then [(0, "s")] else diffs
    where
        diffs :: [(Integer, String)]
        diffs = filter ((/= 0) . fst)
            $ decompose [(86400, "d"), (3600, "h"), (60, "m"), (1, "s")] (floor diff)
        decompose [] _ = []
        decompose ((secs, unit) : metrics) t = 
            let (n, t') = t `divMod` secs
            in (n, unit) : decompose metrics t'
