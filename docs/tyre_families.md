# Road tyre families: research and authored fits

## Player-facing choices

The garage uses technical/style names, not decade labels or a numbered upgrade ladder:

| Family | Inspiration and tread | Grip factors (long./lat.) | Resistance factor | Reference speed | Stiffness factor |
|---|---|---|---|---|---|
| Cord Touring | Early natural-rubber/cotton-cord inspiration; simple rib tread | 0.52 / 0.52 | 1.90 | 120 km/h | 0.65 |
| Belted Touring | Later synthetic compound/belted-bias inspiration; zigzag blocks | 0.68 / 0.68 | 1.50 | 180 km/h | 0.80 |
| Steel Radial | Established steel-belted radial inspiration; conventional blocks | 0.82 / 0.82 | 1.20 | 210 km/h | 0.90 |
| Eco Touring | Modern silica-compound inspiration; fine siping and drainage | 0.87 / 0.87 | 0.80 | 240 km/h | 0.95 |
| Sport Radial | Modern performance-compound inspiration; directional tread | 0.98 / 0.98 | 1.00 | 300 km/h | 1.05 |

Existing Performance Road, Track Semi-Slick, Racing Slick and Drag remain available. The player selects a type card, then a width in the separate submenu. The redundant “Choose width” card text is removed. Historical generations are design inspiration, not UI labels.

## Research basis

- Continental's [milestones](https://www.continental.com/en/company/history/milestones/) describe cord-fabric adoption in 1921, carbon-black reinforcement in 1926 and radial mass production in 1960. Its [technology history](https://www.continental.com/en/company/history/150-years/historical-anniversary-publication/the-continental-tire/) describes cotton cord, steel-belted radial development and later silica compounds. This supports separating construction and compound advances rather than assigning one universal technology to each decade.
- Michelin's [heritage](https://www.michelin.com/en/group/heritage) dates its radial patent to 1946 and silica road-tyre introduction to 1992. [Materials research](https://www.michelin.com/en/expertise/designing-innovative-materials) explains silica's energy-efficiency role.
- Coker's [construction comparison](https://cokertire.com/bias-ply-radial) explains bias/radial cord orientation, tread/sidewall behavior and the overlap of both constructions. Its [classic tyre descriptions](https://cokertire.com/bias-look-radial-tires) show why vintage appearance alone does not identify the underlying construction. The garage art is therefore tread styling, not proof of an actual historical carcass.
- Michelin's [speed-rating chart](https://www.michelin.co.uk/auto/advice/tyre-basics/tyre-load-rating-speed-rating) provides conventional 120/180/210/240/270/300 km/h reference points. Assigning one to a fictional profile is an authored choice, not historical certification.

## Physical implementation and limits

All numbers in the table are authored gameplay fits relative to the pinned baseline tyre. The sources support qualitative distinctions; they do not supply those numerical grip, stiffness or resistance multipliers. The modern compatible tyre dimensions and existing force model are retained. No actual carcass, rubber chemistry, temperature, wear, wet-grip or blowout simulation is added by these names.

Parameters and display order are in data/tyres/families.json. tools/generate_tyre_variants.py creates actual tyre definitions and UI metadata, applying width response plus each family's friction/resistance factors. Stiffness scales the longitudinal pkx1 and lateral pky1 coefficients. Sedan versions retain the existing 270 km/h baseline cap. Speed ratings participate in the existing speed-dependent resistance model; they do not impose a hard speed limiter.

Every family offers the same compatible width ladder, with the body still enforcing axle limits. Existing saved definition IDs and installation/validation behavior are preserved. New images are consistent transparent component renders with different tread geometry; the mounted car mesh still retains its original tread shape.
Verification: native tyre suite passed 235 assertions across 3 cases; garage smoke passed 137 checks including all nine type cards, opening the economy width submenu without installation, installing a physical economy variant, reset and the existing save/Drive flow. The 1920x1080 screenshot was reviewed and the scripts/image loads reported no errors. Logs: out/road_families_unit2.log and out/road_families_garage.log; screenshots: out/road_families_shots.
