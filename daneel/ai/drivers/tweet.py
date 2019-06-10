import tweepy
import yaml

class Tweet:
    def __init__(self, credential_path):
        c = {}
        with open(credential_path, 'r') as f:
            try:
                c = yaml.safe_load(f)['credentials']
            except yaml.YAMLError as exc:
                print("[Tweet] Unable to parse credential YAML file : ", exc)
                raise exc
        self.consumer_key = c['consumer_key']
        self.consumer_secret = c['consumer_secret']
        self.access_token = c['access_token']
        self.access_token_secret = c['access_token_secret']
        self.auth = tweepy.OAuthHandler(self.consumer_key, self.consumer_secret)
        self.auth.set_access_token(self.access_token, self.access_token_secret)
        self.api = tweepy.API(self.auth)

        self.last_response = None

    def tweet(self, text):
        try:
            self.last_response = self.api.update_status(status=text)
        except tweepy.TweepError as exc:
            print("[Tweet] Tweet send failed: ", exc)

    def tweet_in_thread(self, text):
        if self.last_response is not None:
            try:
                self.last_response = self.api.update_status(status=text, in_reply_to_status_id=self.last_response.id,
                                                        auto_populate_reply_metadata=True)
            except tweepy.TweepError as exc:
                print("[Tweet] Tweet send failed: ", exc)
        else:
            self.tweet(text)

