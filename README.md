# MVEyE Project Details:
This is the repository which contains MVEyE project documents, test documents, microcontroller firmware code, QT application source code and binaries.

# Folder Structure 
FirmwareDevelopment	->

			Bin
			Docs
			SourceCode
ProjectDocs		->

			Keep any Common project related Documents here.
QT_Project		->

			App
			Docs
			SourceCode					
TestDocs	        ->

			Keep all test procedure and release notes here.		
README.md    		->

			Contains Git getting started and best practices

# Git Usage Instructions and best practices:

# Download and install git app on to your PC if not already done.
https://gitforwindows.org/

# Go to Local folder where you want to clone the git repository to
Right click mouse and select git bash here and follow the below steps.

# download a repository on GitHub to your Laptop/PC
git clone https://github.com/ChandraMVE/MVEyE.git

# change into the `repo` directory
cd MVEyE

# create a new branch to store any new changes
git branch develop_yourname

# switch to that branch (line of development)
git checkout develop_yourname

# make changes, for example, edit `file1.md` and `file2.md` using the text editor

# check the files that you have modified using status command
git status

# stage the changed files
git add file1.md file2.md

# take a snapshot of the staging area (anything that's been added)
git commit -m "my snapshot"

# push changes to github
git push --set-upstream origin develop_yourname

# Check your commits using log feature of github
git log

