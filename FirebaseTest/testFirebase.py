from firebasin import Firebase
import time, sys



# set up Firebase object and callback function
FIREBASE_URL = 'https://vivid-fire-6558.firebaseio.com/'
latencyTest = Firebase(FIREBASE_URL + 'latencyTest/')

cnt = 0
CNT_MAX = 1000
def on_change(dataSnapshot):
    global cnt, CNT_MAX
    print dataSnapshot.val()

    if cnt < CNT_MAX:
        cnt = cnt + 1
        latencyTest.set(cnt)

latencyTest.on('value', on_change)

# main loop
startTime = time.time()

latencyTest.set(cnt)
latencyTest.waitForInterrupt()

while cnt < CNT_MAX:
    pass

print (time.time() - startTime) / cnt

quit()
