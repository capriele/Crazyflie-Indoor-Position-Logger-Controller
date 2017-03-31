__all__ = ['PopupMenu','DropDownMenu']

from direct.showbase.DirectObject import DirectObject
from direct.gui.DirectGui import DirectButton, DirectFrame, DGG, OnscreenText
from direct.task import Task
from pandac.PandaModules import *
from direct.showbase import ShowBase

SEQUENCE_TYPES=(tuple,list)
atLeast16=PandaSystem.getMajorVersion()*10+PandaSystem.getMinorVersion()>=16
asList=lambda nc: nc if atLeast16 else nc.asList()

def resetModButtons(win=None):
    if win is not None and not win.getProperties().getForeground():
       return
    origMB=ModifierButtons(base.buttonThrowers[0].node().getModifierButtons())
    origMB.allButtonsUp()
    base.buttonThrowers[0].node().setModifierButtons(origMB)

def clampScalar(value, a, b):
    # calling this ought to be faster than calling both min and max
    if a < b:
        if value < a:
            return a
        elif value > b:
            return b
        else:
            return value
    else:
        if value < b:
            return b
        elif value > a:
            return a
        else:
            return value


class DropDownMenu(DirectObject):
  ALeft=0
  ACenter=1
  ARight=2
  ENone=0
  EFade=1
  ESlide=2
  EStretch=3
  PLeft=0
  PRight=1
  PBottom=2
  PTop=3
  parents=( ('a2dBottomLeft','a2dLeftCenter','a2dTopLeft'),
            ('a2dTopRight','a2dRightCenter','a2dBottomRight'),
            ('a2dBottomLeft','a2dBottomCenter','a2dBottomRight'),
            ('a2dTopLeft','a2dTopCenter','a2dTopRight') )

  def __init__(self,items, parent=None,
               sidePad=.0, edgePos=PTop, align=ALeft, effect=ENone,
               buttonThrower=None,
               font=None, baselineOffset=.0,
               scale=.05, itemHeight=1., leftPad=.0, separatorHeight=.5,
               underscoreThickness=1,
               BGColor=(0,0,0,.7),
               BGBorderColor=(1,.85,.4,1),
               separatorColor=(1,1,1,1),
               frameColorHover=(1,.85,.4,1),
               frameColorPress=(0,1,0,1),
               textColorReady=(1,1,1,1),
               textColorHover=(0,0,0,1),
               textColorPress=(0,0,0,1),
               textColorDisabled=(.5,.5,.5,1),
               draggable=False,
               onMove=None
               ):
      '''
      sidePad : additional space on the left and right of the text item
      edgePos : menu bar position on the screen,
                  use DropDownMenu.PLeft, PRight, PBottom, or PTop
      align   : menu items alignment on menu bar,
                  use DropDownMenu.ALeft, ACenter, or ARight
      effect  : the drop down appearance effect,
                  use DropDownMenu.ENone, EFade, ESlide, or EStretch
      draggable : menu bar's draggability status
      onMove : a function which will be called after changing edge position

      Read the remaining options documentation in PopupMenu class.
      '''
      self.parent=parent if parent else getattr(base,DropDownMenu.parents[edgePos][align])
      self.BT=buttonThrower if buttonThrower else base.buttonThrowers[0].node()
      self.menu=self.parent.attachNewNode('dropdownmenu-%s'%id(self))
      self.font=font if font else TextNode.getDefaultFont()
      self.baselineOffset=baselineOffset
      self.scale=scale
      self.itemHeight=itemHeight
      self.sidePad=sidePad
      self.edgePos=edgePos
      self.alignment=align
      self.effect=effect
      self.leftPad=leftPad
      self.underscoreThickness=underscoreThickness
      self.separatorHeight=separatorHeight
      self.BGColor=BGColor
      self.BGBorderColor=BGBorderColor
      self.separatorColor=separatorColor
      self.frameColorHover=frameColorHover
      self.frameColorPress=frameColorPress
      self.textColorReady=textColorReady
      self.textColorHover=textColorHover
      self.textColorPress=textColorPress
      self.textColorDisabled=textColorDisabled
      self.draggable=draggable
      self.onMove=onMove
      self.dropDownMenu=self.whoseDropDownMenu=None

      self.gapFromEdge=gapFromEdge=.008
      texMargin=self.font.getTextureMargin()*self.scale*.25
      b=DirectButton( parent=NodePath(''), text='^|g_', text_font=self.font, scale=self.scale)
      fr=b.node().getFrame()
      b.getParent().removeNode()
      baselineToCenter=(fr[2]+fr[3])*self.scale
      LH=(fr[3]-fr[2])*self.itemHeight*self.scale
      baselineToTop=(fr[3]*self.itemHeight*self.scale/LH)/(1.+self.baselineOffset)
      baselineToBot=LH/self.scale-baselineToTop
      self.height=LH+.01
      l,r,b,t = 0, 5, -self.height, 0
      self.menuBG = DirectFrame( parent=self.menu, frameColor=BGColor,
         frameSize=(l,r,b,t), state=DGG.NORMAL, suppressMouse=1)
      if self.draggable:
         self.setDraggable(1)
      LSborder=LineSegs()
      LSborder.setThickness(2)
      LSborder.setColor(0,0,0,1)
      LSborder.moveTo(l,0,b)
      LSborder.drawTo(r,0,b)
      self.menuBG.attachNewNode(LSborder.create())
      self.itemsParent=self.menu.attachNewNode('menu items parent')

      x=sidePad*self.scale + gapFromEdge
      for t,menuItemsGenerator in items:
          underlinePos=t.find('_')
          t=t.replace('_','')
          b=DirectButton(
              parent=self.itemsParent, text=t, text_font=self.font, pad=(sidePad,0),
              scale=self.scale,
              pos=(x,0,-baselineToTop*self.scale - gapFromEdge),
              text_fg=textColorReady,
              # text color when mouse over
              text2_fg=textColorHover,
              # text color when pressed
              text1_fg=textColorPress,
              # framecolor when pressed
              frameColor=frameColorPress,
              command=self.__createMenu,
              extraArgs=[True,menuItemsGenerator],
              text_align=TextNode.ALeft,
              relief=DGG.FLAT, rolloverSound=0, clickSound=0
              )
          b['extraArgs']+=[b.getName()]
          b.stateNodePath[2].setColor(*frameColorHover) # framecolor when mouse over
          b.stateNodePath[0].setColor(0,0,0,0) # framecolor when ready
          fr=b.node().getFrame()
          b['frameSize']=(fr[0],fr[1],-baselineToBot,baselineToTop)
          self.accept(DGG.ENTER+b.guiId,self.__createMenu,[False,menuItemsGenerator,b.getName()])
          if underlinePos>-1:
             tn=TextNode('')
             tn.setFont(self.font)
             tn.setText(t[:underlinePos+1])
             tnp=NodePath(tn.getInternalGeom())
             underlineXend=tnp.getTightBounds()[1][0]
             tnp.removeNode()
             tn.setText(t[underlinePos])
             tnp=NodePath(tn.getInternalGeom())
             b3=tnp.getTightBounds()
             underlineXstart=underlineXend-(b3[1]-b3[0])[0]
             tnp.removeNode()
             LSunder=LineSegs()
             LSunder.setThickness(underscoreThickness)
             LSunder.moveTo(underlineXstart+texMargin,0,-.7*baselineToBot)
             LSunder.drawTo(underlineXend-texMargin,0,-.7*baselineToBot)
             underline=b.stateNodePath[0].attachNewNode(LSunder.create())
             underline.setColor(Vec4(*textColorReady),1)
             underline.copyTo(b.stateNodePath[1],10).setColor(Vec4(*textColorPress),1)
             underline.copyTo(b.stateNodePath[2],10).setColor(Vec4(*textColorHover),1)
             self.accept('alt-'+t[underlinePos].lower(),self.__createMenu,[True,menuItemsGenerator,b.getName()])
          x+=(fr[1]-fr[0])*self.scale
      self.width=x-2*gapFromEdge
      self.align(align)
      self.setEdgePos(edgePos)
      self.minZ=base.a2dBottom+self.height if edgePos==DropDownMenu.PBottom else None
      viewPlaneNode = PlaneNode('cut menu')
      viewPlaneNode.setPlane( Plane( Vec3( 0, 0, -1 ), Point3( 0, 0, -LH ) ) )
      self.clipPlane=self.menuBG.attachNewNode(viewPlaneNode)

  def __createMenu(self,clicked,menuItemsGenerator,DBname,crap=None):
      itself=self.dropDownMenu and self.whoseDropDownMenu==DBname
      if not (clicked or self.dropDownMenu) or (not clicked and itself): return
      self.__removeMenu()
      if clicked and itself: return
      # removes any context menu
      if clicked:
         self.__removePopupMenu()
      self.dropDownMenu = PopupMenu( items=menuItemsGenerator(),
         parent=self.parent, buttonThrower=self.BT,
         font=self.font, baselineOffset=self.baselineOffset,
         scale=self.scale, itemHeight=self.itemHeight,
         leftPad=self.leftPad, separatorHeight=self.separatorHeight,
         underscoreThickness=self.underscoreThickness,
         BGColor=self.BGColor,
         BGBorderColor=self.BGBorderColor,
         separatorColor=self.separatorColor,
         frameColorHover=self.frameColorHover,
         frameColorPress=self.frameColorPress,
         textColorReady=self.textColorReady,
         textColorHover=self.textColorHover,
         textColorPress=self.textColorPress,
         textColorDisabled=self.textColorDisabled,
         minZ=self.minZ
      )
      self.acceptOnce(self.dropDownMenu.BTprefix+'destroyed',setattr,[self,'dropDownMenu',None])
      self.whoseDropDownMenu=DBname
      item=self.menu.find('**/%s'%DBname)
      fr=item.node().getFrame()
      #~ if self.edgePos==DropDownMenu.PLeft:
         #~ x=max(fr[1],self.dropDownMenu.menu.getX(item))
         #~ z=fr[2]
      #~ elif self.edgePos==DropDownMenu.PRight:
         #~ x=min(fr[0],self.dropDownMenu.menu.getX(item))
         #~ z=fr[2]-self.dropDownMenu.maxWidth
      #~ elif self.edgePos in (DropDownMenu.PBottom,DropDownMenu.PTop):
         #~ x=fr[1]-self.dropDownMenu.maxWidth if self.alignment==DropDownMenu.ARight else fr[0]
         #~ z=fr[2] if self.edgePos==DropDownMenu.PTop else fr[3]+self.dropDownMenu.height
      if self.edgePos==DropDownMenu.PLeft:
         x=max(fr[1],self.dropDownMenu.menu.getX(item))
         z=fr[3]-(self.height-self.gapFromEdge)/self.scale
      elif self.edgePos==DropDownMenu.PRight:
         x=min(fr[0],self.dropDownMenu.menu.getX(item))
         z=fr[3]-(self.height-self.gapFromEdge)/self.scale-self.dropDownMenu.maxWidth
      elif self.edgePos in (DropDownMenu.PBottom,DropDownMenu.PTop):
         x=fr[1]-self.dropDownMenu.maxWidth if self.alignment==DropDownMenu.ARight else fr[0]
         z=fr[3]-(self.height-self.gapFromEdge)/self.scale if self.edgePos==DropDownMenu.PTop else fr[2]+(self.height)/self.scale+self.dropDownMenu.height
      self.dropDownMenu.menu.setPos(item,x,0,z)

      if self.effect==DropDownMenu.EFade:
         self.dropDownMenu.menu.colorScaleInterval(.3,Vec4(1),Vec4(1,1,1,0),blendType='easeIn').start()
      elif self.effect==DropDownMenu.ESlide:
         pos=self.dropDownMenu.menu.getPos()
         if self.edgePos==DropDownMenu.PTop:
            startPos=Point3(0,0,self.dropDownMenu.height*self.scale)
         elif self.edgePos==DropDownMenu.PBottom:
            startPos=Point3(0,0,-self.dropDownMenu.height*self.scale)
         elif self.edgePos==DropDownMenu.PLeft:
            startPos=Point3(-self.dropDownMenu.maxWidth*self.scale,0,0)
         elif self.edgePos==DropDownMenu.PRight:
            startPos=Point3(self.dropDownMenu.maxWidth*self.scale,0,0)
         self.dropDownMenu.menu.posInterval(.3,pos,pos+startPos,blendType='easeIn').start()
         self.dropDownMenu.menu.setClipPlane(self.clipPlane)
      elif self.effect==DropDownMenu.EStretch:
         if self.edgePos==DropDownMenu.PTop:
            startHpr=Vec3(0,-90,0)
         elif self.edgePos==DropDownMenu.PBottom:
            dz=self.dropDownMenu.height*self.scale
            for c in asList(self.dropDownMenu.menu.getChildren()):
                c.setZ(c.getZ()+dz)
            self.dropDownMenu.menu.setZ(self.dropDownMenu.menu,-dz)
            startHpr=Vec3(0,90,0)
         elif self.edgePos==DropDownMenu.PLeft:
            startHpr=Vec3(90,0,0)
         elif self.edgePos==DropDownMenu.PRight:
            dx=self.dropDownMenu.maxWidth*self.scale
            for c in asList(self.dropDownMenu.menu.getChildren()):
                c.setX(c.getX()-dx)
            self.dropDownMenu.menu.setX(self.dropDownMenu.menu,dx)
            startHpr=Vec3(-90,0,0)
         self.dropDownMenu.menu.hprInterval(.3,Vec3(0),startHpr,blendType='easeIn').start()

  def __removeMenu(self):
      if self.dropDownMenu:
         self.dropDownMenu.destroy()
         self.dropDownMenu=None

  def __removePopupMenu(self):
      menuEvents=messenger.find('menu-')
      if menuEvents:
         menuEvent=menuEvents.keys()
         activeMenu=menuEvents[menuEvent[0]].values()[0][0].im_self
         activeMenu.destroy(delParents=True)

  def __reverseItems(self):
      tmp=NodePath('')
      self.itemsParent.getChildren().reparentTo(tmp)
      children=asList(tmp.getChildren())
      for c in reversed(children):
          c.reparentTo(self.itemsParent)
      tmp.removeNode()
      x=self.sidePad*self.scale + self.gapFromEdge
      for c in asList(self.itemsParent.getChildren()):
          c.setX(x)
          fr=c.node().getFrame()
          x+=(fr[1]-fr[0])*self.scale

  def __startDrag(self,crap):
      taskMgr.add(self.__drag,'dragging menu bar',
        extraArgs=[Point2(base.mouseWatcherNode.getMouse())])
      self.__removePopupMenu()
      self.origBTprefix=self.BT.getPrefix()
      self.BT.setPrefix('dragging menu bar')

  def __drag(self,origMpos):
      if base.mouseWatcherNode.hasMouse():
         mpos=base.mouseWatcherNode.getMouse()
         if mpos!=origMpos:
            x,y=mpos.getX(),mpos.getY(),
            deltas=[x+1,1-x,y+1,1-y]
            closestEdge=deltas.index( min(deltas) )
            if closestEdge!=self.edgePos:
               self.setEdgePos(closestEdge)
      return Task.cont

  def __stopDrag(self,crap):
      taskMgr.remove('dragging menu bar')
      self.BT.setPrefix(self.origBTprefix)

  def destroy(self):
      self.__removeMenu()
      self.ignoreAll()
      self.menu.removeNode()

  def isDraggable(self):
      '''
      Returns menu bar's draggable status
      '''
      return self.draggable

  def setDraggable(self,d):
      '''
      Sets menu bar's draggable status
      '''
      self.draggable=d
      if d:
         self.menuBG.bind(DGG.B1PRESS,self.__startDrag)
         self.menuBG.bind(DGG.B1RELEASE,self.__stopDrag)
      else:
         self.menuBG.unbind(DGG.B1PRESS)
         self.menuBG.unbind(DGG.B1RELEASE)

  def align(self,align=None):
      '''
      Aligns menu text on menu bar.
      Use one of DropDownMenu.ALeft, DropDownMenu.ACenter, or DropDownMenu.ARight.
      '''
      self.parent=getattr( base,
         DropDownMenu.parents[self.edgePos][self.alignment if align is None else align])
      self.menu.reparentTo(self.parent)
      if align is not None:
         self.itemsParent.setX(-.5*self.width*align)
         self.menuBG.setX(-.5*(5-self.width)*align)
         self.alignment=align

  def setEdgePos(self,edge):
      '''
      Sticks menu bar to 4 possible screen edges :
          DropDownMenu.PLeft, DropDownMenu.PRight,
          DropDownMenu.PBottom, or DropDownMenu.PTop.
      '''
      lastEdge=self.edgePos
      reverseItems=lastEdge==DropDownMenu.PLeft
      self.edgePos=edge
      self.itemsParent.setZ(0)
      self.menuBG.setSz(1)
      alignment=None
      if self.edgePos==DropDownMenu.PLeft:
         self.menu.setR(-90)
         if self.alignment!=DropDownMenu.ACenter:
            alignment=2-self.alignment
         self.__reverseItems()
         self.minZ=None
      else:
         if self.edgePos==DropDownMenu.PRight:
            self.menu.setR(90)
            self.minZ=None
         elif self.edgePos==DropDownMenu.PBottom:
            self.menu.setR(0)
            self.menuBG.setSz(-1)
            self.itemsParent.setZ(-self.menuBG.node().getFrame()[2])
            self.minZ=base.a2dBottom+self.height
         elif self.edgePos==DropDownMenu.PTop:
            self.menu.setR(0)
            self.minZ=None
         if reverseItems:
            if self.alignment!=DropDownMenu.ACenter:
               alignment=2-self.alignment
            self.__reverseItems()
      self.align(alignment)
      if callable(self.onMove):
         self.onMove()


class PopupMenu(DirectObject):
  '''
  A class to create a popup or context menu.
  Features :
    [1] it's destroyed by pressing ESCAPE, or LMB/RMB click outside of it
    [2] menu item's command is executed by pressing ENTER/RETURN or SPACE when
        it's hilighted
    [3] you can use arrow UP/DOWN to navigate
    [4] separator lines
    [5] menu item image
    [6] menu item hotkey
        If there are more than 1 item using the same hotkey,
        those items will be hilighted in cycle when the hotkey is pressed.
    [7] shortcut key text at the right side of menu item
    [8] multiple lines item text
    [9] menu item can have sub menus
    [10] it's offscreen-proof, try to put your pointer next to screen edge or corner
        before creating it
  '''
  grayImages={} # storage of grayed images,
                # so the same image will be converted to grayscale only once

  def __init__(self,items, parent=None, buttonThrower=None, onDestroy=None,
               font=None, baselineOffset=.0,
               scale=.05, itemHeight=1., leftPad=.0, separatorHeight=.5,
               underscoreThickness=1,
               BGColor=(0,0,0,.7),
               BGBorderColor=(1,.85,.4,1),
               separatorColor=(1,1,1,1),
               frameColorHover=(1,.85,.4,1),
               frameColorPress=(0,1,0,1),
               textColorReady=(1,1,1,1),
               textColorHover=(0,0,0,1),
               textColorPress=(0,0,0,1),
               textColorDisabled=(.5,.5,.5,1),
               minZ=None,
               useMouseZ=True
               ):
      '''
      items : a collection of menu items
         Item format :
            ( 'Item text', 'path/to/image', command )
                        OR
            ( 'Item text', 'path/to/image', command, arg1,arg2,.... )
         If you don't want to use an image, pass 0.

         To create disabled item, pass 0 for the command :
            ( 'Item text', 'path/to/image', 0 )
         so, you can easily switch between enabled or disabled :
            ( 'Item text', 'path/to/image', command if commandEnabled else 0 )
                        OR
            ( 'Item text', 'path/to/image', (0,command)[commandEnabled] )

         To create submenu, pass a sequence of submenu items for the command.
         To create disabled submenu, pass an empty sequence for the command.

         To enable hotkey, insert an underscore before the character,
         e.g. hotkey of 'Item te_xt' is 'x' key.

         To add shortcut key text at the right side of the item, append it at the end of
         the item text, separated by "more than" sign, e.g. 'Item text>Ctrl-T'.

         To insert separator line, pass 0 for the whole item.


      parent : where to attach the menu, defaults to aspect2d

      buttonThrower : button thrower whose thrown events are blocked temporarily
                      when the menu is displayed. If not given, the default
                      button thrower is used

      onDestroy : user function which will be called after the menu is fully destroyed
      
      font           : text font
      baselineOffset : text's baseline Z offset

      scale       : text scale
      itemHeight  : spacing between items, defaults to 1
      leftPad     : blank space width before text
      separatorHeight : separator line height, relative to itemHeight

      underscoreThickness : underscore line thickness

      BGColor, BGBorderColor, separatorColor, frameColorHover, frameColorPress,
      textColorReady, textColorHover, textColorPress, textColorDisabled
      are some of the menu components' color

      minZ : minimum Z position to restrain menu's bottom from going offscreen (-1..1).
             If it's None, it will be set a little above the screen's bottom.
      '''
      self.parent=parent if parent else aspect2d
      self.onDestroy=onDestroy
      self.BT=buttonThrower if buttonThrower else base.buttonThrowers[0].node()
      self.menu=NodePath('menu-%s'%id(self))
      self.parentMenu=None
      self.submenu=None
      self.BTprefix=self.menu.getName()+'>'
      self.submenuCreationTaskName='createSubMenu-'+self.BTprefix
      self.submenuRemovalTaskName='removeSubMenu-'+self.BTprefix
      self.font=font if font else TextNode.getDefaultFont()
      self.baselineOffset=baselineOffset
      self.scale=scale
      self.itemHeight=itemHeight
      self.leftPad=leftPad
      self.separatorHeight=separatorHeight
      self.underscoreThickness=underscoreThickness
      self.BGColor=BGColor
      self.BGBorderColor=BGBorderColor
      self.separatorColor=separatorColor
      self.frameColorHover=frameColorHover
      self.frameColorPress=frameColorPress
      self.textColorReady=textColorReady
      self.textColorHover=textColorHover
      self.textColorPress=textColorPress
      self.textColorDisabled=textColorDisabled
      self.minZ=minZ
      self.mpos=Point2(base.mouseWatcherNode.getMouse())

      self.itemCommand=[]
      self.hotkeys={}
      self.numItems=0
      self.sel=-1
      self.selByKey=False

      bgPad=self.bgPad=.0125
      texMargin=self.font.getTextureMargin()*self.scale*.25
      b=DirectButton( parent=NodePath(''), text='^|g_', text_font=self.font, scale=self.scale)
      fr=b.node().getFrame()
      b.getParent().removeNode()
      baselineToCenter=(fr[2]+fr[3])*self.scale
      LH=(fr[3]-fr[2])*self.itemHeight*self.scale
      imageHalfHeight=.5*(fr[3]-fr[2])*self.itemHeight*.85
      arrowHalfHeight=.5*(fr[3]-fr[2])*self.itemHeight*.5
      baselineToTop=(fr[3]*self.itemHeight*self.scale/LH)/(1.+self.baselineOffset)
      baselineToBot=LH/self.scale-baselineToTop
      itemZcenter=(baselineToTop-baselineToBot)*.5
      separatorHalfHeight=.5*separatorHeight*LH
      LSseparator=LineSegs()
      LSseparator.setColor(.5,.5,.5,.2)

      arrowVtx=[
        (0,itemZcenter),
        (-2*arrowHalfHeight,itemZcenter+arrowHalfHeight),
        (-arrowHalfHeight,itemZcenter),
        (-2*arrowHalfHeight,itemZcenter-arrowHalfHeight),
      ]
      tri=Triangulator()
      vdata = GeomVertexData('trig', GeomVertexFormat.getV3(), Geom.UHStatic)
      vwriter = GeomVertexWriter(vdata, 'vertex')
      for x,z in arrowVtx:
          vi = tri.addVertex(x, z)
          vwriter.addData3f(x, 0, z)
          tri.addPolygonVertex(vi)
      tri.triangulate()
      prim = GeomTriangles(Geom.UHStatic)
      for i in xrange(tri.getNumTriangles()):
          prim.addVertices(tri.getTriangleV0(i),
                           tri.getTriangleV1(i),
                           tri.getTriangleV2(i))
          prim.closePrimitive()
      geom = Geom(vdata)
      geom.addPrimitive(prim)
      geomNode = GeomNode('arrow')
      geomNode.addGeom(geom)
      realArrow=NodePath(geomNode)
      z=-baselineToTop*self.scale-bgPad
      maxWidth=.1/self.scale
      shortcutTextMaxWidth=0
      anyImage=False
      anyArrow=False
      anyShortcut=False
      arrows=[]
      shortcutTexts=[]
      loadPrcFileData('','text-flatten 0')
      for item in items:
          if item:
             t,imgPath,f=item[:3]
             haveSubmenu=type(f) in SEQUENCE_TYPES
             anyArrow|=haveSubmenu
             anyImage|=bool(imgPath)
             disabled=not len(f) if haveSubmenu else not callable(f)
             args=item[3:]
             underlinePos=t.find('_')
             t=t.replace('_','')
             shortcutSepPos=t.find('>')
             if shortcutSepPos>-1:
                if haveSubmenu:
                   print "\nA SHORTCUT KEY POINTING TO A SUBMENU IS NON-SENSE, DON'T YOU AGREE ?"
                else:
                   shortcutText=NodePath( OnscreenText(
                        parent=self.menu, text=t[shortcutSepPos+1:], font=self.font,
                        scale=1, fg=(1,1,1,1), align=TextNode.ARight,
                   ))
                   shortcutTextMaxWidth=max(shortcutTextMaxWidth,abs(shortcutText.getTightBounds()[0][0]))
                   anyShortcut=True
                t=t[:shortcutSepPos]
             else:
                shortcutText=''
             EoLcount=t.count('\n')
             arrowZpos=-self.font.getLineHeight()*EoLcount*.5
             if disabled:
                b=NodePath( OnscreenText(
                     parent=self.menu, text=t, font=self.font,
                     scale=1, fg=textColorDisabled, align=TextNode.ALeft,
                  ))
                # don't pass the scale and position to OnscreenText constructor,
                # to maintain correctness between the OnscreenText and DirectButton items
                # due to the new text generation implementation
                b.setScale(self.scale)
                b.setZ(z)
                maxWidth=max(maxWidth,b.getTightBounds()[1][0]/self.scale)
                if shortcutText:
                   shortcutText.reparentTo(b)
                   shortcutText.setColor(Vec4(*textColorDisabled),1)
                   shortcutText.setZ(arrowZpos)
                   shortcutTexts.append(shortcutText)
             else:
                b=DirectButton(
                    parent=self.menu, text=t, text_font=self.font, scale=self.scale,
                    pos=(0,0,z),
                    text_fg=textColorReady,
                    # text color when mouse over
                    text2_fg=textColorHover,
                    # text color when pressed
                    text1_fg=textColorHover if haveSubmenu else textColorPress,
                    # framecolor when pressed
                    frameColor=frameColorHover if haveSubmenu else frameColorPress,
                    command=(lambda:0) if haveSubmenu else self.__runCommand,
                    extraArgs=[] if haveSubmenu else [f,args],
                    text_align=TextNode.ALeft,
                    relief=DGG.FLAT, rolloverSound=0, clickSound=0, pressEffect=0
                    )
                b.stateNodePath[2].setColor(*frameColorHover) # framecolor when mouse over
                b.stateNodePath[0].setColor(0,0,0,0) # framecolor when ready
                bframe=Vec4(b.node().getFrame())
                if EoLcount:
                   bframe.setZ(EoLcount*10)
                   b['frameSize']=bframe
                maxWidth=max(maxWidth,bframe[1])
                if shortcutText:
                   for snpi,col in ((0,textColorReady),(1,textColorPress),(2,textColorHover)):
                       sct=shortcutText.copyTo(b.stateNodePath[snpi],sort=10)
                       sct.setColor(Vec4(*col),1)
                       sct.setZ(arrowZpos)
                       shortcutTexts.append(sct)
                   shortcutText.removeNode()
             if imgPath:
                img=loader.loadTexture(imgPath)
                if disabled:
                   if imgPath in PopupMenu.grayImages:
                      img=PopupMenu.grayImages[imgPath]
                   else:
                      pnm=PNMImage()
                      img.store(pnm)
                      pnm.makeGrayscale(.2,.2,.2)
                      img=Texture()
                      img.load(pnm)
                      PopupMenu.grayImages[imgPath]=img
                img.setMinfilter(Texture.FTLinearMipmapLinear)
                img.setWrapU(Texture.WMClamp)
                img.setWrapV(Texture.WMClamp)
                CM=CardMaker('')
                CM.setFrame(-2*imageHalfHeight-leftPad,-leftPad, itemZcenter-imageHalfHeight,itemZcenter+imageHalfHeight)
                imgCard=b.attachNewNode(CM.generate())
                imgCard.setTexture(img)
             if underlinePos>-1:
                oneLineText=t[:underlinePos+1]
                oneLineText=oneLineText[oneLineText.rfind('\n')+1:]
                tn=TextNode('')
                tn.setFont(self.font)
                tn.setText(oneLineText)
                tnp=NodePath(tn.getInternalGeom())
                underlineXend=tnp.getTightBounds()[1][0]
                tnp.removeNode()
                tn.setText(t[underlinePos])
                tnp=NodePath(tn.getInternalGeom())
                b3=tnp.getTightBounds()
                underlineXstart=underlineXend-(b3[1]-b3[0])[0]
                tnp.removeNode()
                underlineZpos=-.7*baselineToBot-self.font.getLineHeight()*t[:underlinePos].count('\n')
                LSunder=LineSegs()
                LSunder.setThickness(underscoreThickness)
                LSunder.moveTo(underlineXstart+texMargin,0,underlineZpos)
                LSunder.drawTo(underlineXend-texMargin,0,underlineZpos)
                if disabled:
                   underline=b.attachNewNode(LSunder.create())
                   underline.setColor(Vec4(*textColorDisabled),1)
                else:
                   underline=b.stateNodePath[0].attachNewNode(LSunder.create())
                   underline.setColor(Vec4(*textColorReady),1)
                   underline.copyTo(b.stateNodePath[1],10).setColor(Vec4(*textColorHover if haveSubmenu else textColorPress),1)
                   underline.copyTo(b.stateNodePath[2],10).setColor(Vec4(*textColorHover),1)
                   hotkey=t[underlinePos].lower()
                   if hotkey in self.hotkeys:
                      self.hotkeys[hotkey].append(self.numItems)
                   else:
                      self.hotkeys[hotkey]=[self.numItems]
                      self.accept(self.BTprefix+hotkey,self.__processHotkey,[hotkey])
                      self.accept(self.BTprefix+'alt-'+hotkey,self.__processHotkey,[hotkey])
             if haveSubmenu:
                if disabled:
                   arrow=realArrow.instanceUnderNode(b,'')
                   arrow.setColor(Vec4(*textColorDisabled),1)
                   arrow.setZ(arrowZpos)
                else:
                   arrow=realArrow.instanceUnderNode(b.stateNodePath[0],'r')
                   arrow.setColor(Vec4(*textColorReady),1)
                   arrow.setZ(arrowZpos)
                   arrPress=realArrow.instanceUnderNode(b.stateNodePath[1],'p')
                   arrPress.setColor(Vec4(*textColorHover),1)
                   arrPress.setZ(arrowZpos)
                   arrHover=realArrow.instanceUnderNode(b.stateNodePath[2],'h')
                   arrHover.setColor(Vec4(*textColorHover),1)
                   arrHover.setZ(arrowZpos)
                   # weird, if sort order is 0, it's obscured by the frame
                   for a in (arrPress,arrHover):
                       a.reparentTo(a.getParent(),sort=10)
             if not disabled:
                extraArgs=[self.numItems,f if haveSubmenu else 0]
                self.accept(DGG.ENTER+b.guiId,self.__hoverOnItem,extraArgs)
                self.accept(DGG.EXIT+b.guiId,self.__offItem)
                #~ self.itemCommand.append((None,0) if haveSubmenu else (f,args))
                self.itemCommand.append((f,args))
                if self.numItems==0:
                   self.firstButtonIdx=int(b.guiId[2:])
                self.numItems+=1
             z-=LH + self.font.getLineHeight()*self.scale*EoLcount
          else: # SEPARATOR LINE
             z+=LH-separatorHalfHeight-baselineToBot*self.scale
             LSseparator.moveTo(0,0,z)
             LSseparator.drawTo(self.scale*.5,0,z)
             LSseparator.drawTo(self.scale,0,z)
             z-=separatorHalfHeight+baselineToTop*self.scale
      maxWidth+=7*arrowHalfHeight*(anyArrow or anyShortcut)+.2+shortcutTextMaxWidth
      arrowXpos=maxWidth-arrowHalfHeight
      realArrow.setX(arrowXpos)
      if anyImage:
         leftPad+=2*imageHalfHeight+leftPad
      for sct in shortcutTexts:
          sct.setX(maxWidth-2*(arrowHalfHeight*anyArrow+.2))
      for c in asList(self.menu.findAllMatches('**/DirectButton*')):
          numLines=c.node().getFrame()[2]
          c.node().setFrame(Vec4( -leftPad,maxWidth,
                                  -baselineToBot-(numLines*.1*self.itemHeight if numLines>=10 else 0),
                                  baselineToTop))
      loadPrcFileData('','text-flatten 1')

      try:
         minZ=self.menu.getChild(0).getRelativePoint(b,Point3(0,0,b.node().getFrame()[2]))[2]
      except:
         minZ=self.menu.getChild(0).getRelativePoint(self.menu,Point3(0,0,b.getTightBounds()[0][2]))[2]-baselineToBot*.5
      try:
         top=self.menu.getChild(0).node().getFrame()[3]
      except:
         top=self.menu.getChild(0).getZ()+baselineToTop
      l,r,b,t = -leftPad-bgPad/self.scale, maxWidth+bgPad/self.scale, minZ-bgPad/self.scale, top+bgPad/self.scale
      menuBG = DirectFrame( parent=self.menu.getChild(0),
         frameSize=(l, r, b, t), frameColor=BGColor,
         state=DGG.NORMAL, suppressMouse=1
      )
      menuBorder=self.menu.getChild(0).attachNewNode('border')
      borderVtx=(
        (l,0,b),
        (l,0,.5*(b+t)),
        (l,0,t),
        (.5*(l+r),0,t),
        (r,0,t),
        (r,0,.5*(b+t)),
        (r,0,b),
        (.5*(l+r),0,b),
        (l,0,b),
      )
      LSborderBG=LineSegs()
      LSborderBG.setThickness(4)
      LSborderBG.setColor(0,0,0,.7)
      LSborderBG.moveTo(*(borderVtx[0]))
      for v in borderVtx[1:]:
          LSborderBG.drawTo(*v)
      # fills the gap at corners
      for v in range(0,7,2):
          LSborderBG.moveTo(*(borderVtx[v]))
      menuBorder.attachNewNode(LSborderBG.create())
      LSborder=LineSegs()
      LSborder.setThickness(2)
      LSborder.setColor(*BGBorderColor)
      LSborder.moveTo(*(borderVtx[0]))
      for v in borderVtx[1:]:
          LSborder.drawTo(*v)
      menuBorder.attachNewNode(LSborder.create())
      for v in range(1,8,2):
          LSborderBG.setVertexColor(v,Vec4(0,0,0,.1))
          LSborder.setVertexColor(v,Vec4(.3,.3,.3,.5))
      menuBorderB3=menuBorder.getTightBounds()
      menuBorderDims=menuBorderB3[1]-menuBorderB3[0]
      menuBG.wrtReparentTo(self.menu,sort=-1)
      self.menu.reparentTo(self.parent)
      x=-menuBorderB3[0][0]*self.scale
      for c in asList(self.menu.getChildren()):
          c.setX(x)
      self.maxWidth=maxWidth=menuBorderDims[0]
      self.height=menuBorderDims[2]
      maxWidthR2D=maxWidth*self.menu.getChild(0).getSx(render2d)
      separatorLines=self.menu.attachNewNode(LSseparator.create(),10)
      separatorLines.setSx(maxWidth)
      for v in range(1,LSseparator.getNumVertices(),3):
          LSseparator.setVertexColor(v,Vec4(*separatorColor))
      x=clampScalar( -.98,.98-maxWidthR2D,self.mpos[0]-maxWidthR2D*.5)
      minZ=(-.98 if self.minZ is None else self.minZ)
      z=clampScalar(minZ+menuBorderDims[2]*self.scale*self.parent.getSz(render2d),.98,self.mpos[1] if useMouseZ else -1000)
      self.menu.setPos(render2d,x,0,z)
      self.menu.setTransparency(1)

      self.origBTprefix=self.BT.getPrefix()
      self.BT.setPrefix(self.BTprefix)
      self.accept(self.BTprefix+'escape',self.destroy)
      for e in ('mouse1','mouse3'):
          self.accept(self.BTprefix+e,self.destroy,[True])
      self.accept(self.BTprefix+'arrow_down',self.__nextItem)
      self.accept(self.BTprefix+'arrow_down-repeat',self.__nextItem)
      self.accept(self.BTprefix+'arrow_up',self.__prevItem)
      self.accept(self.BTprefix+'arrow_up-repeat',self.__prevItem)
      self.accept(self.BTprefix+'enter',self.__runSelItemCommand)
      self.accept(self.BTprefix+'space',self.__runSelItemCommand)

  def __offItem(self,crap):
      self.sel=-1
      self.__cancelSubmenuCreation()

  def __hoverOnItem(self,idx,menu,crap):
      self.sel=idx
      self.__cancelSubmenuCreation()
      if self.BT.getPrefix()==self.BTprefix or \
           (self.submenu and self.submenuIdx==idx):
         self.__cancelSubmenuRemoval()
      if menu:
         if not (self.submenu and self.submenuIdx==idx):
            #~ if self.selByKey:
               #~ self.selByKey=False
               #~ self.__createSubmenu(idx,menu)
            #~ else:
               taskMgr.doMethodLater(.3,self.__createSubmenu,self.submenuCreationTaskName,extraArgs=[idx,menu])
      else:
         taskMgr.doMethodLater(.5,self.__removeSubmenu,self.submenuRemovalTaskName,extraArgs=[])

  def __cancelSubmenuCreation(self):
      taskMgr.removeTasksMatching('createSubMenu-*')

  def __createSubmenu(self,idx,menu):
      self.__cancelSubmenuCreation()
      self.__removeSubmenu()
      self.submenu = PopupMenu( items=menu,
         parent=self.parent, buttonThrower=self.BT,
         font=self.font, baselineOffset=self.baselineOffset,
         scale=self.scale, itemHeight=self.itemHeight,
         leftPad=self.leftPad, separatorHeight=self.separatorHeight,
         underscoreThickness=self.underscoreThickness,
         BGColor=self.BGColor,
         BGBorderColor=self.BGBorderColor,
         separatorColor=self.separatorColor,
         frameColorHover=self.frameColorHover,
         frameColorPress=self.frameColorPress,
         textColorReady=self.textColorReady,
         textColorHover=self.textColorHover,
         textColorPress=self.textColorPress,
         textColorDisabled=self.textColorDisabled,
         minZ=self.minZ, useMouseZ=False
      )
      self.submenuIdx=idx
      self.submenu.parentMenu=self
      if self.menu.getBinName():
         self.submenu.menu.setBin(self.menu.getBinName(),self.menu.getBinDrawOrder()+1)
      sb3=self.submenu.menu.getTightBounds()
      sb=sb3[1]-sb3[0]
      b3=self.menu.getTightBounds()
      x=b3[1][0]
      if render2d.getRelativePoint(self.parent,Point3(x+sb[0],0,0))[0]>.98:
         x=b3[0][0]-sb[0]
      if render2d.getRelativePoint(self.parent,Point3(x,0,0))[0]<-.98:
         x=self.parent.getRelativePoint(render2d,Point3(-.98,0,0))[0]
      item=self.menu.find('**/*-pg%s'%(self.firstButtonIdx+idx))
      z=self.parent.getRelativePoint(item,Point3(0,0,item.node().getFrame()[3]))[2]+self.bgPad
      self.submenu.menu.setPos(x,0,max(z,self.submenu.menu.getZ()))
#       self.submenu.menu.setPos(x,0,z)

  def __nextItem(self):
      if self.numItems:
         self.sel=clampScalar(0,self.numItems-1,self.sel+1)
         self.__putPointerAtItem()
         self.selByKey=True

  def __prevItem(self):
      if self.numItems:
         self.sel=clampScalar(0,self.numItems-1,(self.sel-1) if self.sel>-1 else self.numItems-1)
         self.__putPointerAtItem()
         self.selByKey=True

  def __putPointerAtItem(self):
      item=self.menu.find('**/*-pg%s'%(self.firstButtonIdx+self.sel))
      fr=item.node().getFrame()
      c=Point3(.5*(fr[0]+fr[1]),0,.5*(fr[2]+fr[3]))
      cR2D=render2d.getRelativePoint(item,c)
      x,y=int(base.win.getXSize()*.5*(cR2D[0]+1)), int(base.win.getYSize()*.5*(-cR2D[2]+1))
      if '__origmovePointer' in base.win.DtoolClassDict:
         base.win.DtoolClassDict['__origmovePointer'](base.win,0,x,y)
      else:
         base.win.movePointer(0,x,y)

  def __processHotkey(self,hotkey):
      itemsIdx=self.hotkeys[hotkey]
      if len(itemsIdx)==1 and type(self.itemCommand[itemsIdx[0]][0]) not in SEQUENCE_TYPES:
         self.__runCommand(*self.itemCommand[itemsIdx[0]])
      else:
         if self.sel in itemsIdx:
            idx=itemsIdx.index(self.sel)+1
            idx%=len(itemsIdx)
            self.sel=itemsIdx[idx]
         else:
            self.sel=itemsIdx[0]
         self.selByKey=True
         # if it's already there, putting the pointer doesn't trigger the 'enter'
         # event, so just bypass it
         if not (self.submenu and self.submenuIdx==self.sel) and\
              type(self.itemCommand[itemsIdx[0]][0]) in SEQUENCE_TYPES:
            self.__createSubmenu(self.sel,self.itemCommand[itemsIdx[0]][0])
         self.__putPointerAtItem()

  def __doRunCommand(self,f,args):
      self.destroy(delParents=True)
      f(*args)

  def __runCommand(self,f,args):
      if callable(f):
         # must be done at next frame, so shortcut key event won't bleed to the scene
         taskMgr.doMethodLater(.01,self.__doRunCommand,'run menu command',extraArgs=[f,args])

  def __runSelItemCommand(self):
      if self.sel==-1: return
      self.__runCommand(*self.itemCommand[self.sel])

  def __cancelSubmenuRemoval(self):
      taskMgr.removeTasksMatching('removeSubMenu-*')

  def __removeSubmenu(self):
      self.__cancelSubmenuRemoval()
      if self.submenu:
         self.submenu.destroy()

  def destroy(self,delParents=False):
      self.__cancelSubmenuCreation()
      self.__removeSubmenu()
      self.subMenu=None
      self.ignoreAll()
      self.menu.removeNode()
#       if self.origBTprefix.find('menu-')==-1:
#          taskMgr.step()
      self.BT.setPrefix(self.origBTprefix)
      messenger.send(self.BTprefix+'destroyed')
      if delParents and self.parentMenu:
         parent=self.parentMenu
         while parent.parentMenu:
               parent=parent.parentMenu
         parent.destroy()
      if self.parentMenu:
         self.parentMenu.submenuIdx=None
         self.parentMenu=None
      if callable(self.onDestroy):
         self.onDestroy()


'''
if __name__ == '__main__':
   import direct.directbase.DirectStart
   from direct.interval.IntervalGlobal import *
   import os,sys

   os.chdir(sys.path[0])

   modelsPath=filter(lambda p: p.getBasename().find('models')>-1, [getModelPath().getDirectory(i) for i in range(getModelPath().getNumDirectories())])
   getModelPath().appendPath(modelsPath[0].getFullpath()+'/maps')

   thickness=1
   gameSlots={}
   for s in range(4):
       gameSlots[s]=None
   TSmode=(TextureStage.MModulate,TextureStage.MBlend,TextureStage.MDecal)

   def func1():
       model.setR(model,-20)
       print 'func 1'

   def func2(a,b,c):
       model.setR(model,20)
       print 'func 2 args :', a,b,c

   def func3():
       model.setScale(model,.8)
       print 'func 3'

   def func4():
       model.setScale(model,1.25)
       print 'func 4'

   def func5():
       model.clearTransform()
       print 'func 5'

   def addTexture(stage,path):
       tex=loader.loadTexture(path)
       tex.setWrapU(Texture.WMRepeat)
       tex.setWrapV(Texture.WMRepeat)
       s=str(stage)
       TS=model.findTextureStage(s)
       if not TS:
          TS=TextureStage(s)
          TS.setMode(TSmode[stage-2])
          TS.setSort(stage-1)
       model.setTexture(TS,tex)

   def removeTexture(stage):
       TS=model.findTextureStage(str(stage))
       model.clearTexture(TS)

   def newGame():
       global thickness
       thickness=1
       render.setState(RenderState.makeEmpty())
       model.setState(RenderState.makeEmpty())
       func5()
       print 'NEW GAME'

   def loadGame(slot):
       print 'LOADED from slot',slot+1

   def saveGame(slot):
       gameSlots[slot]=1
       print 'SAVED to slot',slot+1

   def changeThickness(t):
       global thickness
       thickness=t
       render.setRenderModeThickness(t)

   def changeModel(newmodel):
       tempParent=NodePath('')
       newModel=loader.loadModel(newmodel)
       newModel.findAllMatches('**/+GeomNode').reparentTo(tempParent)
       tempParent.setScale(1.5/tempParent.getBounds().getRadius())
       tempParent.setPos(-tempParent.getBounds().getCenter())
       tempParent.flattenStrong()
       tempParent.find('**/+GeomNode').node().replaceNode(model.node())

   def exit():
       Sequence(
          Func(gameMenu.destroy),
          model.scaleInterval(.3,model.getScale()*.005,blendType='easeIn'),
          Func(sys.exit)
       ).start()

   def createGameMenuItems():
       return (
       ('_New',0,newGame),
       0, # separator
       ('_Load',0, [ ('slot %s'%(s+1),0,loadGame if gameSlots[s] else 0,s) for s in range(4) ] ),
       ('_Save',0, [ ('slot %s'%(s+1),0,saveGame,s) for s in range(4) ] ),
       0, # separator
       ('Prefe_rence',0,lambda:0),
       0, # separator
       ('E_xit>Escape','lilsmiley.rgba',exit),
       )

   def createViewMenuItems():
       rms=[ (render.setRenderMode if render.getRenderMode()!=rm else 0,rm,thickness,0) \
                for rm in (RenderModeAttrib.MWireframe,
                           RenderModeAttrib.MFilled, RenderModeAttrib.MPoint)
           ]
       return (
       ('_Wireframe mode',0)+rms[0],
       ('_Filled mode',0)+rms[1],
       ('_Point mode',0)+rms[2],
       0, # separator
       ('_Thickness',0, [ ('_%s'%i,0,changeThickness if thickness!=i else 0,i) for i in range(1,10) ]),
       )

   def createModelMenuItems():
       return (
       ('_Abstract room',0,changeModel,'samples/Normal-Mapping/models/abstractroom'),
       ('_Carousel',0,changeModel,'samples/Carousel/models/carousel_base'),
       ('_Eve',0,changeModel,'samples/Looking-and-Gripping/models/eve'),
       ('_Monster',0,changeModel,'samples/Motion-Trails/models/dancer'),
       ('Music _box',0,changeModel,'samples/Music-Box/models/MusicBox'),
       ('_Panda',0,changeModel,'panda-model'),
       ('_Ralph',0,changeModel,'samples/Roaming-Ralph/models/ralph'),
       ('_Smiley',0,changeModel,'smiley'),
       ('_TV man',0,changeModel,'samples/Teapot-on-TV/models/mechman_idle'),
       )

   def createMenuMenuItems():
       effs=[ (0 if gameMenu.effect==X else setattr,gameMenu,'effect',X) \
                for X in (DropDownMenu.ENone, DropDownMenu.EFade,
                          DropDownMenu.ESlide, DropDownMenu.EStretch)
           ]
       algs=[ (0 if gameMenu.alignment==X else gameMenu.align,X) \
                for X in (DropDownMenu.ALeft,
                          DropDownMenu.ACenter, DropDownMenu.ARight)
           ]
       pos=[ (0 if gameMenu.edgePos==X else gameMenu.setEdgePos,X) \
                for X in (DropDownMenu.PLeft, DropDownMenu.PRight,
                          DropDownMenu.PBottom, DropDownMenu.PTop)
           ]
       return (
       ('_Effect',0, (
           ('_None',0)+effs[0],
           ('_Fade',0)+effs[1],
           ('_Slide',0)+effs[2],
           ('_Stretch',0)+effs[3],
       )),
       ('_Alignment',0, (
           ('_Left',0)+algs[0],
           ('_Center',0)+algs[1],
           ('_Right',0)+algs[2],
       )),
       ('_Position',0, (
           ('_Left',0)+pos[0],
           ('_Right',0)+pos[1],
           ('_Bottom',0)+pos[2],
           ('_Top',0)+pos[3],
           0, # separator
           ('you can also do this by\ndragging the menubar',0,0),
       )),
       (['_Draggable','Not _draggable'][gameMenu.isDraggable()],0,gameMenu.setDraggable,not gameMenu.isDraggable())
       )

   def createHelpMenuItems():
       return (
       ('User _guide',0,lambda:0),
       0, # separator
       ('_Register',0,lambda:0),
       ('Report _problems',0,lambda:0),
       ('Request new _features',0,lambda:0),
       ('_Donate authors',0,lambda:0),
       0, # separator
       ('_About',0,lambda:0),
       )

   def createContextMenu():
       myPopupMenu=PopupMenu(
          items=(
            # format :
            #   ( 'Item text', 'path/to/image', command, arg1,arg2,.... )
            # for hotkey, insert an underscore before the character
            ('Rotate 20 deg _CCW',0,func1),
            ('Rotate 20 deg C_W',0,func2, 0,1,2), # appends some arguments (0,1,2)
            0, # separator
            ('_Shrink>Ctrl+S','envir-tree2.png',func3),
            ('_Grow>Ctrl+G','envir-tree1.png',func4),
            0, # separator
            ('_Reset transform','envir-mountain1.png',func5),
            0, # separator
            ('Multitextures stage _2',0, (
                ('Grid','grid.rgb',addTexture,2,'grid.rgb'),
                ('Ground','envir-ground.jpg',addTexture,2,'envir-ground.jpg'),
                ('Rock','envir-rock1.jpg',addTexture,2,'envir-rock1.jpg'),
                ('Toontown map','4map.rgb',addTexture,2,'4map.rgb'),
                0, # separator
                ('_Clear',0,removeTexture if model.findTextureStage('2') else 0,2),
            )),
            ('Multitextures stage _3',0, (
                ('Noise','noise.rgb',addTexture,3,'noise.rgb'),
                ('Controls','shuttle_controls_1.rgb',addTexture,3,'shuttle_controls_1.rgb'),
                ('Text glyphs','cmtt12.rgb',addTexture,3,'cmtt12.rgb'),
                0, # separator
                ('_Clear',0,removeTexture if model.findTextureStage('3') else 0,3),
            )),
            ('Multitextures stage _4',0, (
                ('Tree','envir-tree2.png',addTexture,4,'envir-tree2.png'),
                ('Bamboo','envir-bamboo.png',addTexture,4,'envir-bamboo.png'),
                ('Mountain','envir-mountain2.png',addTexture,4,'envir-mountain2.png'),
                0, # separator
                ('_Clear',0,removeTexture if model.findTextureStage('4') else 0,4),
            )),
            ('Clear _multitextures',0,model.clearTexture if model.findAllTextureStages().getNumTextureStages()>1 else 0),
            0, # separator
            ('_dummy item1 to show hotkey cycle',0,lambda:0),
            ('_dummy item2 to show hotkey cycle>no shortcut',0,lambda:0),
            ('_dummy item3 to show hotkey cycle',0,lambda:0),
            0, # separator
            ("Multiple lines is possible.\n    The arrow is vertically aligned.\n        The _Underline is placed correctly.>this shortcut won't be displayed", 0, (
               ('The shortcut is also\n    vertically aligned>a fake SHORTcut',0,lambda:0),
               ('submenu\n    item #2>me too :p',0,lambda:0)
            )),
            0, # separator
            ('I have some submenus',0, (
               ('submenu item #1',0, (
                  ('submenu #1 subsubmenu item #1',0,lambda:0),
                  ('submenu #1 subsubmenu item #2',0,0),
               )),
               ('submenu item #2',0, (
                  ('submenu #2 subsubmenu item #1',0,lambda:0),
                  ('submenu #2 subsubmenu item #2',0,lambda:0),
                  ('submenu #2 subsubmenu item #3',0, (
                     ('submenu #2 subsubmenu #3 subsubsubmenu item #1',0,lambda:0),
                     ('submenu #2 subsubmenu #3 subsubsubmenu item #2',0,lambda:0),
                     ('submenu #2 subsubmenu #3 subsubsubmenu item #3',0,lambda:0),
                     ('submenu #2 subsubmenu #3 subsubsubmenu item #4',0,lambda:0),
                  )),
                  # pass empty sequence to make a disabled item with submenu
                  ('submenu #2 disabled subsubmenu item #4',0,[]),
               )),
               ('submenu item #3',0, (
                  ('submenu #3 subsubmenu item #1',0,lambda:0),
                  ('submenu #3 subsubmenu item #2',0,0),
                  ('submenu #3 subsubmenu item #3',0,lambda:0),
                  ('submenu #3 subsubmenu item #4',0,lambda:0),
                  ('submenu #3 subsubmenu item #5',0,lambda:0),
                  ('submenu #3 subsubmenu item #6',0,0),
                  ('submenu #3 subsubmenu item #7',0,lambda:0),
               )),
            )),
            # pass empty sequence to make a disabled item with submenu
            ("But I'm disabled",0, []),
            # disabled items, pass 0 for the command
            ('_Disabled item>shortcut1','lilsmiley.rgba',0),
            0, # separator
            ('E_xit>Escape','lilsmiley.rgba',exit),
          ),
          #~ font=loader.loadFont('fonts/Medrano.ttf'),
          baselineOffset=-.35,
          scale=.045, itemHeight=1.2, leftPad=.2,
          separatorHeight=.3,
          underscoreThickness=1,

          BGColor=(.9,.9,.8,.94),
          BGBorderColor=(.8,.3,0,1),
          separatorColor=(0,0,0,1),
          frameColorHover=(.3,.3,.3,1),
          frameColorPress=(0,1,0,.85),
          textColorReady=(0,0,0,1),
          textColorHover=(1,.7,.2,1),
          textColorPress=(0,0,0,1),
          textColorDisabled=(.65,.65,.65,1),

          # just another color scheme
          #~ BGColor=(0,0,0,.94),
          #~ BGBorderColor=(1,.85,.4,1),
          #~ separatorColor=(1,1,1,1),
          #~ frameColorHover=(1,.85,.4,1),
          #~ frameColorPress=(0,1,0,1),
          #~ textColorReady=(1,1,1,1),
          #~ textColorHover=(0,0,0,1),
          #~ textColorPress=(0,0,0,1),
          #~ textColorDisabled=(.5,.5,.5,1),
       )

   def menuBarMoved():
       print 'menuBarMoved'

   model=loader.loadModel('smiley').find('**/+GeomNode')
   model.reparentTo(render.attachNewNode('modelParent'))
   model.setTransparency(1)
   model.getParent().hprInterval(5,Vec3(360,0,0)).loop()

   gameMenu = DropDownMenu(
       items=(
         ('_Game', createGameMenuItems),
         ('_View', createViewMenuItems),
         ('Mo_del', createModelMenuItems),
         ('_Menu', createMenuMenuItems),
         ('_Help', createHelpMenuItems)
       ),
       sidePad=.75,
       align=DropDownMenu.ALeft,
       #~ align=DropDownMenu.ACenter,
       #~ align=DropDownMenu.ARight,
       #~ effect=DropDownMenu.ESlide,
       effect=DropDownMenu.EStretch,
       #~ effect=DropDownMenu.EFade,
       edgePos=DropDownMenu.PTop,
       #~ edgePos=DropDownMenu.PBottom,
       #~ edgePos=DropDownMenu.PLeft,
       #~ edgePos=DropDownMenu.PRight,

       #~ font=loader.loadFont('fonts/Medrano.ttf'),
       baselineOffset=-.35,
       scale=.045, itemHeight=1.2, leftPad=.2,
       separatorHeight=.3,
       underscoreThickness=1,

       BGColor=(.9,.9,.8,.94),
       BGBorderColor=(.8,.3,0,1),
       separatorColor=(0,0,0,1),
       frameColorHover=(.3,.3,.3,1),
       frameColorPress=(0,1,0,.85),
       textColorReady=(0,0,0,1),
       textColorHover=(1,.7,.2,1),
       textColorPress=(0,0,0,1),
       textColorDisabled=(.65,.65,.65,1),
       draggable=True,
       onMove=menuBarMoved
    )

   OnscreenText( parent=base.a2dBottomCenter,
                 text='Right click to pop up a context menu', fg=(1,1,1,1),
                 pos=(0,.02), scale=.045)

   DO=DirectObject()
   DO.accept('mouse3',createContextMenu)
   DO.accept('control-s',func3)
   DO.accept('control-g',func4)
   DO.accept('escape',exit)
   DO.accept('window-event',resetModButtons)

   camera.setY(-5)
   base.disableMouse()
   run()
'''