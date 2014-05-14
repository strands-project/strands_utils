#!/usr/bin/python

import sys
import ftplib

def moveFTPFiles(serverName,userName,passWord,remotePath,remoteFolder,localPath):
	"""Connect to an FTP server and bring down files to a local directory"""
	import os
	from sets import Set
	from ftplib import FTP

	# open FTP connection, login and cd to the correct directory
	print "\n-- Connecting to FTP ----\n"
	try:
		ftp = FTP(serverName)
	except:
		print "Couldn't find server"
		return
	try:
		ftp.login(userName,passWord)
	except: 
		print 'Could not log in'
		return
	try:
		ftp.cwd(remotePath)
	except:
		try:
			ftp.mkd(remotePath)
			ftp.cwd(remotePath)
		except:
			print 'Remote directory does not exist and it cannot be created.'
			return
	# create remote directory, including subdirectories
	index = 0
	while  index != -1:
		index = remoteFolder.find('/')
		if index != -1:
			intFolder = remoteFolder[:index]
			remoteFolder = remoteFolder[index+1:]
		else: 
			intFolder = remoteFolder
		rFileSet = Set(ftp.nlst())
		if not intFolder in rFileSet:
			try:
				ftp.mkd(intFolder)
				ftp.cwd(intFolder)
				print 'Creating folder ',intFolder,' on FTP'
			except:
				print 'Cannot create remote folder'
				return
	
	# cd to the correct directory on the local system
	try:
		os.chdir(localPath)
	except:
		print 'Cannot cd to local path'
		return

	localFiles = []
	for (dirpath, dirnames, filenames) in os.walk(localPath):
	    localFiles.extend(filenames)
	    break

	print "\n-- Uploading Files----\n"
	# upload local files to remote folder
 	files_copied = 0
	for filename in localFiles:
		try:
			print 'Uploading file ', filename, ' to FTP'
			ftp.storlines('STOR ' + filename, open(filename, 'r'))
			files_copied+= 1			
		except:	
			print 'Could not upload file ',filename

	print 'Uploaded ',files_copied,' out of ', len(localFiles), ' files.'


if __name__ == '__main__':
	
	if len(sys.argv) < 7:
		print 'Please provide 6 arguments: Ftp server name, user, password, remote directory path, remote folder, local directory path'
	else:		
	        #--- connection variables
        	ftpServerName = sys.argv[1]
	        ftpU = sys.argv[2]
        	ftpP = sys.argv[3]
	        remoteDirectoryPath = sys.argv[4]
        	remoteFolder = sys.argv[5]
	        localDirectoryPath = sys.argv[6]      

	        deleteLocalFolder = False
        	deleteLocalFolderParent = False
	        moveFTPFiles(ftpServerName,ftpU,ftpP,remoteDirectoryPath,remoteFolder, localDirectoryPath)


