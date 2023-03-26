import board
import simpleio

musicpin = board.GP6


'''
How melodies are transposed into code that is played on buzzer:
Every song needs two lists, one to store the variables for the notes and octaves,
and one to store the corresponding lengths of the beats for each note.
The length of both lists MUST be the same. Numbers are used to indicate the note length.
In the case of most melodies:
8 = eighth note
4 = quarter note
2 = half note
1 = whole note
'''

# Music note frequencies ( with columns as octaves)
REST    = 0
NOTE_C  = [0, 32, 65, 131, 262, 523, 1047]
NOTE_CS = [0, 34, 69, 139, 277, 554, 1109]
NOTE_D  = [0, 36, 73, 147, 294, 587, 1175]
NOTE_DS = [0, 37, 78, 156, 311, 622, 1245]
NOTE_E  = [0, 41, 82, 165, 330, 659, 1319]
NOTE_F  = [0, 43, 87, 175, 349, 698, 1397]
NOTE_FS = [0, 46, 92, 185, 370, 740, 1480]
NOTE_G  = [0, 49, 98, 196, 392, 784, 1568]
NOTE_GS = [0, 52, 104, 208, 415, 831, 1661]
NOTE_A  = [0, 55, 110, 220, 440, 880, 1760]
NOTE_AS = [0, 58, 117, 223, 466, 932, 1865]
NOTE_B  = [0, 61, 123, 246, 492, 984, 1976]

# Songs
# https://github.com/robsoncouto/arduino-songs/blob/master/miichannel/miichannel.ino
mii_theme_notes = [NOTE_FS[4], REST, NOTE_A[4], NOTE_CS[5], REST, NOTE_A[4],
                   REST, NOTE_FS[4], NOTE_D[4], NOTE_D[4], NOTE_D[4], REST,
                   REST, REST, NOTE_CS[4], NOTE_D[4], NOTE_FS[4], NOTE_A[4],
                   NOTE_CS[5], REST, NOTE_A[4], REST, NOTE_F[4], NOTE_E[5],
                   NOTE_DS[5], NOTE_D[5], REST, REST, NOTE_GS[4], REST,
                   NOTE_CS[5], NOTE_FS[4], REST, NOTE_CS[5], REST, NOTE_GS[4],
                   REST, NOTE_CS[5], REST, NOTE_G[4], NOTE_FS[4], REST,
                   NOTE_E[4], REST, NOTE_E[4], NOTE_E[4], NOTE_E[4], REST,
                   REST, NOTE_E[4], NOTE_E[4], NOTE_E[4], REST, REST,
                   NOTE_DS[4], NOTE_D[4], NOTE_CS[4], REST]
mii_theme_beats = [8, 8, 8, 8, 8, 8,
                   8, 8, 8, 8, 8, 8,
                   4, 8, 8, 8, 8, 8,
                   8, 8, 8, 8, 8, 3,
                   7, 8, 8, 8, 4, 8,
                   8, 8, 8, 8, 8, 8,
                   8, 8, 8, 8, 8, 8,
                   8, 8, 8, 8, 8, 8,
                   8, 4, 8, 8, 8, 8,
                   8, 8, 8, 8]

#https://github.com/robsoncouto/arduino-songs/blob/master/starwars/starwars.ino
starwars_notes = [NOTE_AS[4], NOTE_AS[4], NOTE_AS[4], NOTE_F[5], NOTE_C[6], NOTE_AS[5],
                  NOTE_A[5], NOTE_G[5], NOTE_F[6], NOTE_C[6], NOTE_AS[5], NOTE_A[5],
                  NOTE_G[5], NOTE_F[6], NOTE_C[6], NOTE_AS[5], NOTE_A[5], NOTE_AS[5],
                  NOTE_G[5], NOTE_C[5], NOTE_C[5], NOTE_C[5], NOTE_F[5], NOTE_C[6],
                  NOTE_AS[5], NOTE_A[5], NOTE_G[5], NOTE_F[6], NOTE_C[6], NOTE_AS[5],
                  NOTE_A[5], NOTE_G[5], NOTE_F[6], NOTE_C[6], NOTE_AS[5], NOTE_A[5],
                  NOTE_AS[5], NOTE_G[5]]  
starwars_beats = [8, 8, 8, 2, 2, 8, 
                  8, 8, 2, 4, 8, 8,
                  8, 2, 4, 8, 8, 8,
                  2, 8, 8, 8, 2, 2,
                  8, 8, 8, 2, 4, 8,
                  8, 8, 2, 4, 8, 8,
                  8, 2]

class MusicPlayer:
    def __init__(self, pin=musicpin):
        self.pin=pin
        self.songlist = {'mii':[mii_theme_notes, mii_theme_beats],
                         'starwars':[starwars_notes, starwars_beats]
                        }
        
    def play(self, songname):
        print('received songname', songname)
        if songname in self.songlist.keys():
            print("File ",songname, " found in library. Playing now...")
            self.playsong(self.songlist[songname][0], self.songlist[songname][1])


    def playsong(self, songnotes, songbeats, tempo=2):
        '''
        songnotes: list of the melodies notes
        songbeats: list of melodies beat times
        tempo: speed of song, this is not traditional tempo in bpm like on a metronome, 
        but more like a multiplier for whatever the notes are so a tempo value of 2 
        make it play twice as fast. Adjust this by ear.
        
        This function plays the melody, simply by iterating through the list. 
        '''
        for i in range(0, len(songnotes)):
            simpleio.tone(self.pin, songnotes[i], (1/songbeats[i])*tempo)
        simpleio.tone(self.pin, 0, 0.1)
    
    
    