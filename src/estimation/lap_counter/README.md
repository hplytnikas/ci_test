# Lap counter
## Subscriptions

The node fetch transforms from map -> base_link to compute how many laps has the car done so far

## Fetch:
map -> base_link

## Publish:
/lap_count

## Explanation:

The code fetch the current pose of the car from the trasnformation map -> base_link. When the car reach the starting point it publish the current number of laps performed and the time taken to performe the lap
