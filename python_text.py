from Tkinter import *


class PythonText(Text):
    """ Tk Text Widget that supports some Python syntax coloring"""

    def __init__(self, *args, **kwargs):
        Text.__init__(self, *args, **kwargs)
        self.patterns={}
        self.bind("<KeyRelease>",self.colorize)

        self.tag_configure("comment",foreground="#007f00")
        self.tag_configure("keyword",foreground="#0000Af")
        self.tag_configure("string",foreground="#7f007f")

        self.addPattern("#.*","comment")
        self.addPattern("'.*'","string")
        self.addPattern('".*"',"string")
        keywords=["if","else","elif","for","while","def","class","pass"]
        for k in keywords:
            self.addPattern(k,"keyword")
        
        

    def addPattern(self,regexp,tag):
        self.patterns[regexp]=tag
    
    def colorize(self,*args,**kwargs):        
        for regexp in self.patterns.keys():
            self.highlight_pattern(regexp,self.patterns[regexp],regexp=True)
        
    def highlight_pattern(self, pattern, tag, start="1.0", end="end", regexp=False):
        '''Apply the given tag to all text that matches the given pattern

        If 'regexp' is set to True, pattern will be treated as a regular expression
        '''

        start = self.index(start)
        end = self.index(end)
        self.mark_set("matchStart",start)
        self.mark_set("matchEnd",start)
        self.mark_set("searchLimit", end)

        count = IntVar()
        while True:
            index = self.search(pattern, "matchEnd","searchLimit",
                                count=count, regexp=regexp)
            if index == "": break
            self.mark_set("matchStart", index)
            self.mark_set("matchEnd", "%s+%sc" % (index,count.get()))
            self.tag_add(tag, "matchStart","matchEnd")