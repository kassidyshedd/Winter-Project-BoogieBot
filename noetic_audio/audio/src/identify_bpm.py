import rospy 
from std_msgs.msg import String, Int64
from enum import Enum, auto

# Add for bpm detection
import asyncio
from shazamio import Shazam
import spotipy
from spotipy.oauth2 import SpotifyOAuth

class State(Enum):
    WAITING = auto(),
    IDENTIFYING = auto(),
    DONE = auto()

class IdentifyBPM:
    def __init__(self):
        rospy.init_node('identify_bpm')
        self.state = State.WAITING

        # Publishers
        self.num_tempo_pub = rospy.Publisher("num_tempo", Int64, queue_size=10)

        # Subscribers
        self.start_sub = rospy.Subscriber('start', String, self.start_callback)

        #Initialize
        self.first_message = False

    def start_callback(self, msg):
        if not self.first_message:
            self.state = State.IDENTIFYING
            self.first_message = True
            rospy.loginfo("Identify - switch state to idenitfy")

    async def main(self):

        # ########## Begin Citation [7] ##########
        shazam = Shazam()
        rospy.loginfo("Identify - Identifying song from audio file")
        out = await shazam.recognize_song('data/cupid_shuffle.wav')
        title = out['track']['title']
        artist = out['track']['subtitle']
        print(title, "by", artist)
        # ########## End Citation [7] ##########

        # ########## Begin Citation [5], [6] ###########
        if title and artist:
            sp = spotipy.Spotify(auth_manager=SpotifyOAuth(client_id='',
                                                       client_secret='',
                                                       redirect_uri='http://localhost:8888/callback',
                                                       scope='user-library-read'))
        
            results = sp.search(q=f'artist:{artist} track:{title}', type='track', limit=1)

            if 'tracks' in results and 'items' in results['tracks']:
                if results['tracks']['items']:
                    track_id = results['tracks']['items'][0]['id']
                    # print("Spotify track id:", track_id)   

            audio = sp.audio_features(track_id)
            if audio and isinstance(audio, list):
                get_inst = audio[0]
                tempo = get_inst['tempo']
                print("Unrounded tempo:", tempo)

        tempo_ = round(tempo)
        print("beats per minute", tempo_)
        print("\n")

        # ########## End Citation [5], [6] ##########


    loop = asyncio.get_event_loop()
    loop.run_until_complete(main())


                     

    async def run(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if self.state == State.WAITING:
                pass
            elif self.state == State.IDENTIFYING:
                rospy.loginfo("Identify State")
                BEAT = await self.main()
                self.num_tempo_pub.publish(BEAT)
                rospy.loginfo("Identify - Published integer message")
                self.state = State.DONE
            elif self.state == State.DONE:
                pass
                
            rate.sleep()

    def identify_bpm(self):
        rospy.loginfo("Identify - determining bpm")
        import random
        return random.randint(60, 120)
        
    
if __name__ == '__main__':
    try:
        node = IdentifyBPM()
        asyncio.run(node.run())
    except rospy.ROSInterruptException:
        pass