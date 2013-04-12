-- Create standalone solution
if EVDS_STANDALONE ~= false then
  SIMC_STANDALONE = false
  solution "evds"
     dofile("./../external/simc/support/premake4_common.lua")
     dofile("./../external/simc/support/premake4.lua")
end


--------------------------------------------------------------------------------
-- EVDS documentation
--------------------------------------------------------------------------------
newaction {
   trigger     = "evdsdoc",
   description = "Generate documentation for EVDS",
   execute     = function()
     if EVDS_STANDALONE ~= false
     then os.chdir("../doc")
     else os.chdir("./../external/evds/doc")
     end
     os.execute("doxygen")
   end
}
if _ACTION == "evdsdoc" then return end


--------------------------------------------------------------------------------
-- Generate EVDS default materials include
--------------------------------------------------------------------------------
local evds_defmat = "../include/evds_database.h"
local evds_matxml = "../source/evds_common/evds_database.xml"
if true then -- (not os.isfile(evds_defmat))
  -- Pad XML with C strings
  local xml_contents = ""
  for line in io.lines(evds_matxml) do
    local line = string.gsub(line,"\r","")
    xml_contents = xml_contents.."\t\""..string.gsub(line,"\"","'").."\\n\"\n"
  end
  
  -- Generate source code
  local source = "char* EVDS_InternalCommon_Database = \n"..xml_contents..";\n"

  -- Save source code
  local file = io.open(evds_defmat,"w+")
  file:write(source)
  file:close()
end

newaction {
   trigger     = "evdsdb",
   description = "Generate materials database lookup",
   execute     = function () end
}
if _ACTION == "evdsdb" then return end


--------------------------------------------------------------------------------
-- External Vessel Dynamics Simulator
--------------------------------------------------------------------------------
project "evds"
   library()
   language "C"
   includedirs { "../include",
                 "../external/simc/include" }
   files { "../source/evds_core/**",
           "../source/evds_common/**",
           "../include/**" }
   defines { "EVDS_LIBRARY", "SIMC_LIBRARY" }
   links { "simc" }


--------------------------------------------------------------------------------
-- Tutorials
--------------------------------------------------------------------------------
project "evds_tutorials"
   kind "ConsoleApp"
   language "C"
   includedirs { "../include",
                 "../external/simc/include",
                 "../source/evds_tutorials" }
   files { "../source/evds_tutorials/evds_tutorials.c" }
   links { "evds" } --, "glfw" }
   
--   configuration { "windows" }
--     links { "opengl32" }
   configuration { "not windows" }
      links { "simc", "tinyxml" }
