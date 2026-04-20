# jsk_spot_sing_and_dance

This demo enables spot to sing and dance.

## sing_and_dance.py

With this script, Spot will play music and dance in specified rate.

### Parameters

- `~music_list`
  + Dictionary of music to play
  + Keys of each entry must be music name
  + Each entry must have `filepath` and `bpm` fields.
    - `filepath`: filepath to music file
    - `bpm`: bpm of music

- `~music_name`
  + The name of music to play
