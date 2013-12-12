"""
    (C) 2013 Humarobotics
    
    Provides classes for quick threading of object methods
    
    In your class, just add this line to the __init__:
        self.post=Post(self)
        
    You can now call any object methods with object.post.method(args)
    The Thread object is returned.
    
"""


from threading import Thread

class PostThread(Thread):
    """ Object that manages the threaded execution of a given function """
    
    def __init__(self,func):
        """ Creates the thread object with the method to be called """
        Thread.__init__(self)
        self.func=func
        self.isRunning=True
        self.result=None
    
    def execute(self,*args,**kwargs):
        """ Store the method call arguments and start the thread, returns the thread object to the caller """
        self.args=args
        self.kwargs=kwargs
        self.start()
        return self

    def run(self):
        """ Thread execution, call the function with saved arguments, saves result and change flag at the end """
        self.result=self.func(*self.args,**self.kwargs)
        self.isRunning=False

class Post:
    """ Object that provides threaded calls to its parent object methods """
    def __init__(self,parent):
        self.parent=parent
    
    def __getattr__(self,attr):
        """ Find the method asked for in parent object, encapsulate in a PostThread object and send back pointer to execution function"""
        try:
            func=getattr(self.parent,attr)
            post_thread=PostThread(func)
            return post_thread.execute 
        except:
            raise Exception("ERROR: Post call on %s: method %s not found"%(str(self.parent),attr))


if __name__=="__main__":
    class Dummy:
        def __init__(self):
            self.post=Post(self)
        
        def do(self,param):
            import time
            print "Doing... param="+str(param)
            time.sleep(2)
            print "Done"
            return param
            

    dummy=Dummy()
    dummy.do("direct1")
    dummy.post.do("post1")
    dummy.post.do("post2")
    t3=dummy.post.do("post3")
    t3.join()
    print t3.result
    print "Finished"
    