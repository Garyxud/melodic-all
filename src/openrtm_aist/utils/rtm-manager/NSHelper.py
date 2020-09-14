#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  @file RtmNSHelper.py
#  @brief rtc-link name service access helper class
#  @date $Date: 2005-05-12 09:06:19 $
#  @author Noriaki Ando <n-ando@aist.go.jp>
# 
#  Copyright (C) 2004-2005
#      Task-intelligence Research Group,
#      Intelligent Systems Research Institute,
#      National Institute of
#          Advanced Industrial Science and Technology (AIST), Japan
#      All rights reserved.
# 
#  $Id: RtmNSHelper.py 775 2008-07-28 16:14:45Z n-ando $
# 

import sys
import omniORB.CORBA as CORBA
import CosNaming

class NSHelper:
    def __init__(self):
        self.orb = None
        
#    def __del__(self):
#        if self.orb != None:
#            try:
#                self.orb.destroy()
#            except:
#                except_mess('CORBA Name Server: Destroy Error!! <' \
#				    + name_server + '>')

    def Connect(self, name_server):
        try:
            if self.orb != None:
                self.orb.destroy()
            arg = ["-ORBInitRef",
		   "NameService=corbaname::",
		   "-ORBclientCallTimeOutPeriod",
		   "2000"]
            arg[1] = arg[1] + name_server
            self.orb = CORBA.ORB_init(arg)
            obj = self.orb.resolve_initial_references("NameService")
            self.root_cxt = obj._narrow(CosNaming.NamingContext)
        except:
            self.root_cxt = None
#            print 'CORBA Name Server: Connect Error!! <',name_server,'>'

    def GetNSDict(self):
        ns_dict = None
        if self.root_cxt != None:
            ns_dict = self.__GetNameTreeRecursive__(self.root_cxt)
        return ns_dict

    def __GetNameTreeRecursive__(self, cxt):
        ns_dict = {}
        try:
            cur_cxt = cxt.list(100)
        except:
            except_mess("cxt.list method error!:")
            return
    
        for bc in cur_cxt[0]:
            if bc != None:
                try:
                    cur_obj = cxt.resolve(bc.binding_name)
                    dict_key = bc.binding_name[0].id \
                               + "|" \
                               + bc.binding_name[0].kind
                    if bc.binding_type == CosNaming.ncontext:
                        ns_dict[dict_key] = (
                            {
                            "objref":cur_obj,
                            "id":bc.binding_name[0].id,
                            "kind":bc.binding_name[0].kind,
                            "bname":bc.binding_name
                            },
                            self.__GetNameTreeRecursive__(cur_obj))
                    elif bc.binding_type == CosNaming.nobject:
                        ns_dict[dict_key] = (cur_obj, None)
                        ns_dict[dict_key] = (
                            {
                            "objref":cur_obj,
                            "id":bc.binding_name[0].id,
                            "kind":bc.binding_name[0].kind,
                            "bname":bc.binding_name
                            },None)
                except:
                    except_mess("context method error!:")
                    ns_dict = {}
                    break

        return ns_dict
    
    def DeleteToContext(self,cxt,bname):
        if cxt != None:
            try:
                cxt.unbind(bname)
            except:
                except_mess('contex not found:')


if __name__ == '__main__':
    import sys
    nsh = RtmNSHelper()
    if len(sys.argv) == 1:
        print sys.argv[0] + " [CosNameService Host Name]"
        sys.exit(1)
        
    nsh.Connect(sys.argv[1])
    print nsh.GetNSDict()
        
