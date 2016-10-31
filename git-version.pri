#
# Extract GIT version to be included into any application that wants to know it
#
unix {
    GIT_INFO_commit=    $$system(git rev-parse HEAD)
    GIT_INFO_branch=    $$system(git rev-parse --abbrev-ref HEAD)
    GIT_INFO_untracked= $$system(echo `git status --porcelain 2>/dev/null | grep \"^??\" | wc -l` 'untracked')
    GIT_INFO_uncommited=$$system(echo `git status --porcelain 2>/dev/null | grep \"^M\" | wc -l` 'uncommited')
    GIT_INFO_unadded=   $$system(echo `git status --porcelain 2>/dev/null | grep \"^ M\" | wc -l` 'not added')
    GIT_INFO_allSubs=$$system(git submodule foreach -q 'echo $path `git rev-parse HEAD` `git rev-parse --abbrev-ref HEAD` "(" `git status --porcelain 2>/dev/null | grep -E "^??" | wc -l` " untracked / " `git status --porcelain 2>/dev/null | grep -E "^M" | wc -l` " uncommited /" `git status --porcelain 2>/dev/null | grep -E "^ M" | wc -l` " not added)"')
#    GIT_INFO_allSubs=   $$system(git submodule foreach -q 'echo $path `git rev-parse HEAD` `git rev-parse --abbrev-ref HEAD` "(" `git status --porcelain 2>/dev/null | grep "^??" | wc -l` " untracked / " `git status --porcelain 2>/dev/null | grep "^M" | wc -l` " uncommited /" `git status --porcelain 2>/dev/null | grep "^ M" | wc -l` " not added)"' | tr '\n' ' ')

} else:win32 {

    GitProg = "C:\\Program Files\\Git\\bin\\git.exe"
    !exists($$GitProg) {
        GitProg = "C:\\Program Files (x86)\\Git\\bin\\git.exe"
    }
    !exists($$GitProg) {
        GitProg = "git.exe"
    }
    !exists($$GitProg) {
        message(warning: could not find git.exe in standard path to get the branch version!)
    } else {
        GIT_INFO_commit=    $$system(\"$$GitProg\" rev-parse HEAD)
        GIT_INFO_branch=    $$system(\"$$GitProg\" rev-parse --abbrev-ref HEAD)
		GIT_INFO_untracked= $$system("for /f %i in ('git status --porcelain 2^>nul ^| find /c \"??\"')  do @echo %i untracked")
		GIT_INFO_uncommited=$$system("for /f %i in ('git status --porcelain 2^>nul ^| find /c \" M\"')  do @echo %i uncommited")
		GIT_INFO_unadded=   $$system("for /f %i in ('git status --porcelain 2^>nul ^| find /c \"M  \"') do @echo %i not added")
        GIT_INFO_allSubs=   $$system(\"$$GitProg\" submodule)       # without extra info for each submodule for a while  //TODO:
    }
}
git_info = "GIT_VERSION=\"$$GIT_INFO_commit $$GIT_INFO_branch ($$GIT_INFO_untracked / $$GIT_INFO_uncommited / $$GIT_INFO_unadded) [$$GIT_INFO_allSubs]\""

DEFINES += $$git_info
message($$git_info)
