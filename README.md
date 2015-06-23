# NuttX-L21
NuttX support of the SAM-L21

## Work flow

To contribute please 'fork' this repo and work on branches within your clone.
Submit Pull requests back to this repo.

While under development follow the nuttx coding standard.
 * Use spaces not tabs
 * Use /nuttx/tools/indent.sh 
 * Check to ensure the structure and inclusions of the source file you generate follows that of one Greg has done recently 
     @temas please list some here
 * When submitting a PR if the need be please rebase prior to removed unnecessary work history.

## Branch usages

master   - current merge down - always builds, is as stable as it can be and always can be rebased on upstream

baseline - Used for test and integration will replace master at our release rate.

            To create the new baseline one would branch from nuttx_patches, then rebase off a fresh upstream
              ```
              git branch -m last_baseline last_baseline_<date in yyyymmmdd> (see below)
              git branch -m baseline last_baseline (see below)
              git checkout nuttx_patches
              git checkout -b baseline
              git rebase upstream
              
            ```
last_baseline -  The last baseline before this one. (see above)

last_baseline_<date in yyyymmmdd> - created with ``` -m baseline last_baseline_<date in yyyymmmdd>```

upstream - This branch will equal to that of the Source Forge repo http://git.code.sf.net/p/nuttx/git at the getfilament 
           uptake rate.
           
           The only operations that should be done on this branch are::
           
           ```
           [ Needed Once] git add remote upstream http://git.code.sf.net/p/nuttx/git
           git fetch upstream
           git checkout upstream
           git pull upstream master
           ```
           Never submit a PR against this branch.
  
  upstream_to_greg - This branch will contain the *accepted* changes that have been submitted by team members that will 
                      be submittted upstream to Greg using he mandated submission methods: Emailed patches.
  
  nuttx_patches    - This branch will have changes needed by NuttX-L21 - that will never be accepted upstream.
  
  travis           - used to rebase on master or any branch to run a travis build on.

 
## Nuttx Build
  
  Please run 
  ```
  cd tools;./configure.sh saml21-xplained/nsh;cd ..
  make oldconfig
  make menuconfig
  ```
  Then diff .config back against configs/saml21-xplained/nsh/defconfig - if the changes are **not** related to build platform please copy the use ```cp .config configs/saml21-xplained/nsh/defconfig``` to update the config file - then revert the line ```CONFIG_APPS_DIR="../apps"``` to ```# CONFIG_APPS_DIR="../apps"``` 
  
  The same operations should be done on the ```configs/sigmamodule/defconfig``` but the reversion of the  ```CONFIG_APPS_DIR``` line is not needed unless the config will be submitted upstream
  
