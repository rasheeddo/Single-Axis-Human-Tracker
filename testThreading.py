import threading
import time
def thread1():
	while True:
		print("thread 1 is working")
		time.sleep(0.1)

def thread2():
	while True:
		print("thread 2 is also working....")
		time.sleep(0.5)

try:
	t1=threading.Thread(target=thread1)
	t2=threading.Thread(target=thread2)
	t1.start()
	t2.start()

except:
	print("Error: unable to start thread")
	
while 1:
	pass

