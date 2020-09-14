class MyClass(GeneratedClass):
  def __init__(self):
    try: # disable autoBind;
      GeneratedClass.__init__(self, False);
    except TypeError: # if NAOqi < 1.14;
      GeneratedClass.__init__( self );

  def onLoad(self):
    self.bIsRunning = False;
    self.leds = ALProxy("ALLeds")

  def onUnload(self):
    self.onInput_onStop(); # will stop current loop execution

  def onInput_onStart(self):
    #self.logger.info( self.getName() + ": start - begin" );

    if( self.bIsRunning ):
      #print( self.getName() + ": already started => nothing" );
      return;

    self.bIsRunning = True;
    self.bMustStop = False;

    rDuration = 0.2;
    self.leds.post.fadeRGB( "FaceLedsTop", 0xff00ff, rDuration );
    self.leds.post.fadeRGB( "FaceLedsInternal", 0xff00ff, rDuration );
    self.leds.post.fadeRGB( "FaceLedsBottom", 0xff00ff, rDuration );
    self.leds.fadeRGB( "FaceLedsExternal", 0xff00ff, rDuration );

    while( not self.bMustStop ):
      rTime = 0.1;
      self.leds.post.fadeRGB( "FaceLedsTop", 0xffffff, rTime );
      self.leds.fadeRGB( "FaceLedsBottom", 0xffffff, rTime );
      if( self.bMustStop ):
        break;
      rTime = 0.3
      self.leds.post.fadeRGB( "FaceLedsTop", 0xff00ff, rTime );
      self.leds.fadeRGB( "FaceLedsBottom", 0xff00ff, rTime );


    # end while
    self.bIsRunning = False;
    self.onStopped();

  def onInput_onStop(self):
    self.bMustStop = True; # will stop current loop execution
