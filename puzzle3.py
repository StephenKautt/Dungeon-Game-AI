import math

#the binarydepthfirst search function
def amBDFS(initial, r, c, depth):
    moves = {
        1: (1, -1),
        2: (1, 0),
        3: (1, 1),
        4: (0, -1),
        6: (0, 1),
        7: (-1, -1),
        8: (-1, 0),
        9: (-1, 1),
        "N": (-1, 0),
        "S": (1, 0),
        "E": (0, 1),
        "W": (0, -1)
    }
    depthHit = False
    frontier = []
    push(frontier, [])
    bullet = 1
    score = 50
    ogres, demons = enemyCounter(initial)
    tot = ogres + demons
    while len(frontier) != 0:
        copy = customcopy(initial)
        current = pop(frontier)
        if len(current) == depth:
            check = moveOnState(r, c, current, moves, copy)
            ogresb, demonsb = enemyCounter(check)
            totb = ogresb + demonsb
            if isGoal(check, 10, current, r, c):
                monstdiff = tot - totb
                for x in current:
                    if x in range(1, 10):
                        score -= 1
                    elif (x == "N" or x == "E" or x == "W" or x == "S"):
                        score -= 20
                score += (5*monstdiff)
                for y in current:
                    print(str(y), end="")
                print("")
                print(score)
                printGameState(r, c, check)
                return current
            valid = possMoves(moves, check, bullet, r, c)
            if valid != None:
                depthHit = True
        else:
            check = moveOnState(r, c, current, moves, copy)
            valid = possMoves(moves, check, bullet, r, c)
            if 'N' in current or 'S' in current or 'W' in current or 'E' in current:
                    if 'N' in valid:
                        valid.remove('N')
                    if 'S' in valid:
                        valid.remove('S')
                    if 'E' in valid:
                        valid.remove('E')
                    if 'W' in valid:
                        valid.remove('W')
            for y in valid:
                appender = current[:]
                appender.append(y)
                push(frontier, appender)
    return depthHit

#function for the depth limiter a.k.a the iterative deepening
def iterativeDeepening(initial, r, c):
    depth = 0
    notBool = True
    while notBool:
        res = amBDFS(initial, r, c, depth)
        if isinstance(res, bool):
            if res == False:
                notBool = False
        elif isinstance(res, list):
            return res
        depth += 1

#push and pop from the list to simulate a LIFO stack
def push(stack, item):
    stack.append(item)

def pop(stack):
    if not stack:
        return None
    else:
        return stack.pop()

#compares the board state to a winner one
def isGoal(state, maxturns, movelist, r, c):
    amL = actManPos(state, r, c)
    turn = len(movelist)
    ogres, demons = enemyCounter(state)
    if amL == None:
        return False
    if ogres == 0 and demons == 0 and state[amL[0]][amL[1]] != "X":
        return True
    if turn == maxturns and state[amL[0]][amL[1]] != "X":
        return True
    return False

#counts how many ogres and demons there are
def enemyCounter(state):
    # always in order ogres, demons
    ocount = sum(row.count("G") for row in state)
    dcount = sum(row.count("D") for row in state)
    return ocount, dcount

#a list of possible moves from AM's current position
def possMoves(dic, state, pewpew, r, c):
    bulletNum = pewpew
    moves = []
    shoots = ['N', 'S', 'E', 'W']
    amL = actManPos(state, r, c)
    if amL == None:
        return moves
    for key in dic:
        if key in range(1, 10):
            bruh = list(dic.get(key))
            testCoordx = amL[0] + bruh[0]
            testCoordY = amL[1] + bruh[1]
            if state[testCoordx][testCoordY] != "#":
                moves.append(key)
        elif key in shoots:
            shoots.remove(key)
            moves.append(key)
        else:
            continue
    return moves

#deep copy of the initial board since python built in copy is slow as balls
def customcopy(state):
    if isinstance(state, list):
        ret = []
        for i in state:
            ret.append(customcopy(i))
    elif isinstance(state, (int, float, type(None), str, bool)):
        ret = state
    else:
        raise ValueError("Unexpected type for mydeepcopy function")
    return ret

#takes a likst of moves and returns the board outcome after all moves have passed
def moveOnState(r, c, move, dic, copy):
    bullet = 1
    score = 50
    state = copy
    amL = actManPos(copy, r, c)
    if move == []:
        return state
    ogreL = []
    demonL = []
    corpseL = []
    findEnemyPos(state, r, c, ogreL, demonL, corpseL)
    for x in move:
        if x in range(1, 10):
            if score >= 1:
                score -= 1
                amL = moveAM(x, amL, dic, state)
        elif (x == "N" or x == "E" or x == "W" or x == "S") and bullet > 0 and score >= 20:
            bullet -= 1
            score -= 20
            score += 5*(shoot(x, amL, dic, state, ogreL, demonL, corpseL))
        # monster Move
        monsterRemove(state, ogreL, demonL)
        newOgre = []
        for ogreP in ogreL:
            newOgrePos = ogreMash(state, ogreP, amL, r, c)
            if newOgrePos is not None:
                newOgre.append(newOgrePos)
            else:
                newOgre.append(ogreP)
        ogreL = newOgre

        newDemon = []
        for demonP in demonL:
            newDemonPos = demonMash(state, demonP, amL, r, c)
            if newDemonPos is not None:
                newDemon.append(newDemonPos)
            else:
                newDemon.append(demonP)
        demonL = newDemon

        monsterAdd(state, ogreL, demonL)
        score = monsterCollideCheck(ogreL, demonL, state, amL, corpseL, score)
        if amL in demonL or amL in ogreL or amL in corpseL:
            score = 0
            return state
        if state[amL[0]][amL[1]] == 'X':
            score = 0
            return state
        for demon in demonL:
            if demon in ogreL or demon in corpseL:
                demonL.remove(demon)
            if demon == amL:
                demonL.remove(demon)
        for ogre in ogreL:
            if ogre in demonL or ogre in corpseL:
                ogreL.remove(ogre)
            if ogre == amL:
                ogreL.remove(ogre)
    return state

#finds AM's current position on the board
def actManPos(state, r, c):
    for x in range(r):
        for y in range(c):
            if state[x][y] == "A":
                coord = [x, y]
                return coord

#creates lists of all the enemy positions, ogres, demons, and corpses
def findEnemyPos(state, r, c, ogre, demon, corpse):
    for x in range(r):
        for y in range(c):
            if state[x][y] == "G":
                ogre.append([x, y])
            if state[x][y] == "D":
                demon.append([x, y])
            if state[x][y] == "@":
                corpse.append([x, y])
    return

#moves AM according to the move
def moveAM(a, coord, dic, gameState):
    if gameState[coord[0] + dic[a][0]][coord[1] + dic[a][1]] != "#":
        gameState[coord[0]][coord[1]] = " "
        coord[0] += dic[a][0]
        coord[1] += dic[a][1]
        if gameState[coord[0]][coord[1]] == "G" or gameState[coord[0]][coord[1]] == "D" or gameState[coord[0]][coord[1]] == "@":
            gameState[coord[0]][coord[1]] = "X"
        else:
            gameState[coord[0]][coord[1]] = "A"

    return [coord[0], coord[1]]

#shoots the bullet in the direction provided
def shoot(a, coord, dic, gameState, shrek, muzan, deadAsHell):
    killed = 0
    tempcoord = [coord[0], coord[1]]
    xChange = tempcoord[0] + dic[a][0]
    yChange = tempcoord[1] + dic[a][1]
    while 0 <= xChange < len(gameState) and 0 <= yChange < len(gameState[0]) and gameState[xChange][yChange] != "#":
        if gameState[xChange][yChange] == "G":
            killed += 1
            gameState[xChange][yChange] = "@"
            deadAsHell.append([xChange, yChange])
            if [xChange, yChange] in shrek:
                shrek.remove([xChange, yChange])
        elif gameState[xChange][yChange] == "D":
            killed += 1
            gameState[xChange][yChange] = "@"
            deadAsHell.append([xChange, yChange])
            if [xChange, yChange] in muzan:
                muzan.remove([xChange, yChange])
        xChange += dic[a][0]
        yChange += dic[a][1]
    return killed

#removes monsters from the board before placing them at their new positions
def monsterRemove(gameState, shrek, muzan):
    for x in shrek:
        gameState[x[0]][x[1]] = " "
    for y in muzan:
        gameState[y[0]][y[1]] = " "
    return

#moves the ogres
def ogreMash(gameState, oCoord, amCoord, gSX, gSY):
    possibleo = getAdjacentCells(oCoord, gSX, gSY)
    distanceMino = float('inf')
    besto = None
    clockwise = [[-1, 0], [-1, 1], [0, 1], [1, 1],
                 [1, 0], [1, -1], [0, -1], [-1, -1]]
    bestcounto = []
    for moveo in possibleo:
        dg = oCoord
        if gameState[moveo[0]][moveo[1]] != "#":
            disto = euclidDistance(moveo, amCoord)
            if distanceMino == None:
                distanceMino = disto
            elif disto < distanceMino:
                distanceMino = disto
                besto = moveo
                bestcounto = [x-y for x, y in zip(moveo, dg)]
            elif disto == distanceMino:
                checko = [x-y for x, y in zip(moveo, dg)]
                if clockwise.index(checko) < clockwise.index(bestcounto):
                    besto = moveo
                    bestcounto = checko
    return besto

#euclidian distance to decide best monster move
def euclidDistance(xy1, xy2):
    return math.dist(xy2, xy1)

#check adjacent cells to calculate possible moves for demons and ogres
def getAdjacentCells(xy, c, r):
    adjacent = []
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            if (dx != 0 or dy != 0) and 0 <= xy[0] + dx < c and 0 <= xy[1] + dy < r:
                adjacent.append([xy[0] + dx, xy[1] + dy])
    return adjacent

#moves demons
def demonMash(gameState, dCoord, amCoord, gsX, gsY):
    possible = getAdjacentCells(dCoord, gsX, gsY)
    distanceMin = float('inf')
    best = None
    countClockwise = [[-1, 0], [-1, -1], [0, -1],
                      [1, -1], [1, 0], [1, 1], [0, 1], [-1, 1]]
    bestcount = []
    for move in possible:
        og = dCoord
        if gameState[move[0]][move[1]] != "#":
            dist = euclidDistance(move, amCoord)
            if distanceMin == None:
                distanceMin = dist
            elif dist < distanceMin:
                distanceMin = dist
                best = move
                bestcount = [x-y for x, y in zip(move, og)]
            elif dist == distanceMin:
                check = [x-y for x, y in zip(move, og)]
                if countClockwise.index(check) < countClockwise.index(bestcount):
                    best = move
                    bestcount = check
    return best

#checks all the collision possibilities between the enemy types
def monsterCollideCheck(shrek, muzan, gameState, coord, deadAsHell, num):
    for x in shrek:
        if shrek.count(x) > 1:
            gameState[x[0]][x[1]] = "@"
            deadAsHell.append(x)
            for p in shrek:
                if p == x:
                    shrek.remove(x)
        if x in muzan:
            gameState[x[0]][x[1]] = "@"
            if x in shrek:
                shrek.remove(x)
                muzan.remove(x)
            if x not in deadAsHell:
                deadAsHell.append(x)
        if x in deadAsHell:
            if x in shrek:
                shrek.remove(x)
                gameState[x[0]][x[1]] = "@"
    for y in muzan:
        if muzan.count(y) > 1:
            gameState[y[0]][y[1]] = "@"
            deadAsHell.append(y)
            for h in muzan:
                if h == y:
                    muzan.remove(y)
        if y in deadAsHell:
            if y in muzan:
                muzan.remove(y)
                gameState[y[0]][y[1]] = "@"
    if ((coord in shrek) or (coord in muzan) or (coord in deadAsHell)) and coord != "X":
        gameState[coord[0]][coord[1]] == "X"
        num = 0
    return num

#adds monsters back to the board
def monsterAdd(gameState, shrek, muzan):
    for x in shrek:
        if gameState[x[0]][x[1]] != "X":
            if gameState[x[0]][x[1]] == "A":
                gameState[x[0]][x[1]] = "X"
            else:
                gameState[x[0]][x[1]] = "G"
    for y in muzan:
        if gameState[y[0]][y[1]] != "X":
            if gameState[y[0]][y[1]] == "A":
                gameState[y[0]][y[1]] = "X"
            else:
                gameState[y[0]][y[1]] = "D"
    return

#prints board
def printGameState(r, c, gameState):
    for x in range(r):
        for y in range(c):
            print(gameState[x][y], end="")
        print()
# -----------------------------------------------------------------------------------------------


# initial creator
rows, columns = [int(x) for x in input().split()]
board = []
for p in range(rows):
    row = input()
    append = []
    for q in row:
        append.append(q)
    board.append(append)

#start the function
iterativeDeepening(board, rows, columns)
