{-# OPTIONS -Wall #-}

import Vis

{- 7 precedence
    right-associative scalar-vector multiply: scale vector by preceding scalar
    right scalar-vector divide
    left outer product
    foldr: f z [x1, x2, ..., xn] right
        x1 f(x2 f(...(xn f z)...))
    $ map eField cds: apply eField to each charge in list cds charges data
        yield list of individual electric field
    fld r | fld <- flds: output expr | generator
        a list of fld r vals -}
infixl 6 ^+^
infixl 7 *^
infixl 7 ^/
infixl 7 <.>
infixl 7 ><

type R = Double
showDouble :: R -> String
showDouble x
    | x < 0     = "(" ++ show x ++ ")"
    | otherwise = show x

data Mass = Mass R 
            deriving (Eq, Show)
data Vec = Vec {xComponent :: R, yComponent :: R, zComponent :: R}
            deriving (Eq)
instance Show Vec where
    show (Vec x y z) = "vec " ++ showDouble x ++ " "
                            ++ showDouble y ++ " "
                            ++ showDouble z
vec :: R -> R -> R -> Vec
vec = Vec

negateV :: Vec -> Vec
negateV (Vec ax ay az) = Vec (-ax) (-ay) (-az)
zeroV :: Vec
zeroV = vec 0 0 0
sumV :: [Vec] -> Vec
sumV = foldr (^+^) zeroV
(*^) :: R -> Vec -> Vec
c *^ Vec ax ay az = Vec (c*ax) (c*ay) (c*az)
(^/) :: Vec -> R -> Vec
Vec ax ay az ^/ c = Vec (ax/c) (ay/c) (az/c)
(<.>) : Vec -> Vec -> R
Vec ax ay az <.> Vec bx by bz = ax*bx + ay*by + az*bz
(><) :: Vec -> Vec -> Vec
Vec ax ay az >< Vec bx by bz = Vec (ay*bz - az*by) (az*bx - ax*bz) (ax*by - ay*bx)

magnitude :: Vec -> R
magnitude v = sqrt(v <.> v)

simulateVis :: HasTime s => R -> Int -> s -> (s -> Vis.VisObj R) -> (TimeStep -> s -> s) -> IO ()
simulateVis timeScaleFactor rate initialState picFunc updFunc
    = let visUpdFunc ta st
            = let dtp = tsFactor * realToFrac ta - timeOf st
                in updFunc dtp st
        in Vis.simulate Vis.defaultOpts (1/fromIntegral rate)
        initialState (orient . picFunc) visUpdFunc

pfsVisObj :: R -> ParticleFieldState -> Vis.VisObj R

animatePFS :: R -> Int -> R -> ParticleFieldState -> IO ()
animatePFS tsFactor animationRate width st
    = simulateVis tsFactor animationRate st (pfsVisObj width) pfsUpd

data Position = Cart R R R
                deriving (Show)
cart :: R -> R -> R -> Position
cart = Cart

type ScalarField = Position -> R
origin :: Position
origin = cart 0 0 0

mu0 :: R
mu0 = 4e-7 * pi
cSI :: R
cSI = 299792458
epsilon0 :: R
epsilon0 = 1/(mu0 * cSI**2)

type Charge = R
elementaryCharge :: Charge
elementaryCharge = 1.602176634e-19

data ChargeDist
    = PointCharge   Charge      Position
    | LineCharge    ScalarField Curve
    | SurfaceCharge ScalarField Surface
    | VolumeCharge  ScalarField Volume
    | MultipleCharges [ChargeDist]
protonOrigin :: ChargeDist
protonOrigin = PointCharge elementaryCharge origin

type VectorField = Position -> Vec
type CurveApprox = Curve -> [(Position, Vec)]
type SurfaceApprox = Surface -> [(Position, Vec)]
type VolumeApprox = Volume -> [(Position, R)]
type VectorLineIntegral = VectorField -> Curve -> Vec
type VectorSurfaceIntegral = VectorField -> Surface -> Vec
type VectorVolumeIntegral = VectorField -> Volume -> Vec

vectorLineIntegral :: CurveApprox -> VectorField -> Curve -> Vec
vectorLineIntegral approx vF c
    = sumV [vF r' ^* magnitude dl' | (r', dl') <- approx c]
vectorSurfaceIntegral :: SurfaceApprox -> VectorField -> Surface -> Vec
vectorSurfaceIntegral approx vF s
    = sumV [vF r' ^* magnitude da' | (r', da') <- approx s]
vectorVolumeIntegral :: VolumeApprox -> VectorField -> Volume -> Vec
vectorLineIntegral approx vF vol
    = sumV [vF r' ^* magnitude dv' | (r', dv') <- approx v]

eFieldFromPointCharge :: Charge -> Position -> VectorField
eFieldFromPointCharge q1 r1 r 
    = let k = 1 / (4 * pi * epsilon0)
            d = displacement r1 r
    in (k * q1) *^ d ^/ magnitude d ** 3
eFieldFromLineCharge :: ScalarField -> Curve -> VectorField
eFieldFromLineCharge lamda c r
    = let k = 1 / (4 * pi * epsilon0)
        integrand r' = lamda r' *^ d ^/ magnitude d ** 3
            where d = displacement r' r
    in k *^ vectorLineIntegral (curveSample 1000) integrand c
eFieldFromSurfaceCharge :: ScalarField -> Surface -> VectorField
eFieldFromSurfaceCharge sigma s r
    = let k = 1 / (4 * pi * epsilon0)
        integrand r' = sigma r' *^ d ^/ magnitude d ** 3
            where d = displacement r' r
    in k *^ vectorSurfaceeIntegral (surfaceSample 200) integrand s
eFieldFromVolumeCharge :: ScalarField -> Volume -> VectorField
eFieldFromVolumeCharge rho c r
    = let k = 1 / (4 * pi * epsilon0)
        integrand r' = rho r' *^ d ^/ magnitude d ** 3
            where d = displacement r' r
    in k *^ vectorVolumeIntegral (volumeSample 50) integrand v

addVectorFields :: [VectorField] -> VectorField
addVectorFields flds r = sumV [fld r | fld <- flds]

eField :: ChargeDist -> VectorField
eField (PointCharge     q   r) = eFieldFromPointCharge      q   r
eField (LineCharge      lam c) = eFieldFromLineCharge       lam c
eField (SurfaceCharge   sig s) = eFieldFromSurfaceCharge    sig s
eField (VolumeCharge    rho s) = eFieldFromVolumeCharge     rho v
eField (MultipleCharges     cds) = addVectorFields $ map eField cds


main :: IO ()
main = animatePFS period 30 (4*bohrRadius)
        ( defaultPFS {   mass       = electronMass
                        , charge    = -elementaryCharge
                        , position  = cart bohrRadius 0 0 
                        , velocity  = vec 0 v0 0
                        , electricField = eField protonOrigin })
            where electronMass = 9.109e-31
                    bohrRadius = 0.529e-10
                    v0 = elementaryCharge / sqrt (4*pi*epsilon0*electronMass*bohrRadius)
                    period = 2*pi*bohrRadius/v0


