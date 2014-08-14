#!/usr/bin/python

import sys
import os
from sets import Set
from ftplib import FTP

def moveFTPFiles(serverName,userName,passWord,remotePath,remoteFolder,localPath, deleteAfterUpload):
	"""Connect to an FTP server and bring down files to a local directory"""

	# open FTP connection, login and cd to the correct directory
	print "\n-- Connecting to FTP ----\n"
	try:
		ftp = FTP(serverName)
	except:
		print "Couldn't find server"
		return False
	try:
		ftp.login(userName,passWord)
	except: 
		print 'Could not log in'
		return False
	try:
		ftp.cwd(remotePath)
	except:
		try:
			ftp.mkd(remotePath)
			ftp.cwd(remotePath)
		except:
			print 'Remote directory does not exist and it cannot be created.'
			return False
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
				print 'Creating folder ',intFolder,' on server'
			except:
				print 'Cannot create remote folder'
				return False
		else: 
			ftp.cwd(intFolder)

	print "\n-- Uploading Files----\n"
	retVal = uploadFolder(ftp,remotePath,remoteFolder,localPath, deleteAfterUpload)	
	print "\n-- Finished uploading ----\n"
	return retVal

def uploadFolder(ftp,remotePath,remoteFolder,localPath,deleteAfterUpload):
	
	# cd to the correct directory on the local system
	try:
		os.chdir(localPath)
	except:
		print 'Cannot cd to local path', localPath
		return False
	
	localFiles = []
	localDirs = []
	for (dirpath, dirnames, filenames) in os.walk(localPath):
	    localFiles.extend(filenames)
	    localDirs.extend(dirnames)
	    break


	# upload local files to remote folder
 	files_copied = 0
	for filename in localFiles:
		try:
			print 'Uploading file ', filename, ' to server'
			ftp.storbinary('STOR ' + filename, open(filename, 'rb'))
			files_copied+= 1			
		except:	
			print 'Could not upload file ',filename
		if deleteAfterUpload:
			print 'deleting file ', os.path.join(localPath, filename)
			os.remove(os.path.join(localPath, filename))

	print 'Uploaded ',files_copied,' out of ', len(localFiles), ' files from folder ',localPath

	# recursively upload files from child folders
	for directory in localDirs:
	       # create directory on the FTP
		rFileSet = Set(ftp.nlst())
		if not directory in rFileSet:
			try:
				ftp.mkd(directory)
				ftp.cwd(directory)
				print 'Creating folder ',directory,' on server'
			except:
				print 'Cannot create remote folder'
				return False
		else: 
			ftp.cwd(directory)
		newRemotePath = remotePath+'/'+directory
		newRemoteFolder = remoteFolder + '/'+directory
		newLocalPath = localPath+'/'+directory
		uploadFolder(ftp, newRemotePath, newRemoteFolder, newLocalPath,deleteAfterUpload)
		ftp.cwd('..')
		if deleteAfterUpload:
			print 'deleting folder ', newLocalPath
			os.rmdir(newLocalPath)
	
	return True
				

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


