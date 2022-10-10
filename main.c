#include <stdio.h>

#define MAZESIZE_X	(16)		//迷路の大きさ(MAZESIZE_X * MAZESIZE_Y)迷路
#define MAZESIZE_Y	(16)		//迷路の大きさ(MAZESIZE_X * MAZESIZE_Y)迷路

#define INF 0xffff

#define TRUE 1
#define FALSE 0

#define NOWALL	0	//0 壁がない場合の値
#define WALL	1	//1 壁がある場合の値

#define FAST_SEACH	1	//一定速度での走行の経路導出
#define EXACT_SEACH	0	//加減速を行う走行の経路導出

#define EN_DIA		//斜め方向加減速有効

//#define EN_PRINT_0
//#define EN_PRINT_1

//#define STRAIGHT 90
//#define DIAGONAL 64


typedef struct
{
	unsigned char north;	//北の壁情報
	unsigned char east;	//東の壁情報
	unsigned char south;	//南の壁情報
	unsigned char west;	//西の壁情報
}twall;			//壁情報を格納する構造体

typedef struct
{
	unsigned char x;	//
	unsigned char y;	//
	unsigned char dir;	//
}tindex;			//

tindex index[MAZESIZE_X][MAZESIZE_Y][2]; //経路復元用、次のノードを記録する

twall wall[MAZESIZE_X][MAZESIZE_Y];	//壁の情報を格納する構造体配列
unsigned short		map[MAZESIZE_X][MAZESIZE_Y];	//歩数マップ


tindex location[200];

const short ex_straight[16] ={82,82,164,227,273,311,344,374,404,434,464,494,524,554,584,614}; 
const short ex_diagonalt[32]={58,58,116,174 ,231 ,279 ,315 ,346 ,372 ,397 ,
							419 ,440 ,461 ,483 ,504 ,525 ,546 ,567 ,589 ,610 ,
							631 ,652 ,673 ,695 ,716 ,737 ,758 ,779 ,801 ,822 ,843 ,864 }; 

unsigned char goalx,goaly,goaldir;
unsigned short COST[MAZESIZE_X][MAZESIZE_Y][2];

int dijkstra(int,char);
void setwall(void);
void showmap(void);

int main(void){

	int nodex,nodey,nodedir;
	int tmpx,tmpy,tmpdir;
	int ret;

	//迷路情報の読み込み
	setwall();


	ret = dijkstra(0xff,EXACT_SEACH);
	/* ダイクストラ法で最短経路(の時間)を求める */
	printf("start(0,0:0)からgoal(%d,%d:%d)までの距離: %d ms\n",goalx,goaly,goaldir, ret );

	if(ret==INF){return 0;}

	
	/* 通過ノードを表示する */
	printf("start(0,0:0)からgoal(%d,%d:%d)までに通過する経路(node)\n",goalx,goaly,goaldir);
	nodex = 0;
	nodey = 0;
	nodedir = 0;
	printf("%d,%d:%d", nodex,nodey,nodedir);
	while(1){
		tmpx=nodex;
		tmpy=nodey;
		tmpdir=nodedir;
		nodex = index[tmpx][tmpy][tmpdir].x;
		nodey = index[tmpx][tmpy][tmpdir].y;
		nodedir = index[tmpx][tmpy][tmpdir].dir;
		printf(" -> %d,%d:%d", nodex,nodey,nodedir);
		if (nodex == goalx && nodey==goaly && nodedir==goaldir) break;
	}
	printf("\n");


	printf("\n");
	showmap();//迷路と導出した経路とを表示する
	
	
	#if 0 
	//迷路上に各ノードのcostを表示する
	for(int wy=(MAZESIZE_Y-1);wy>=0 ;wy--){
		for(int wx=0 ;wx<=(MAZESIZE_X-1);wx++){	
			if(wall[wx  ][wy  ].north == WALL   ){printf("+-----");}
			if(wall[wx  ][wy  ].north == NOWALL ){printf("+%4x ",COST[wx][wy][0] );}
		} 
		printf("+\n");
		for(int wx=0 ;wx<=(MAZESIZE_X-1);wx++){
				if(wx==0){
					if(wall[wx  ][wy  ].west  == WALL   ){printf("|  ");}
				}else{
					if(wall[wx  ][wy  ].west  == WALL   ){printf("   |  ");}
					if(wall[wx  ][wy  ].west  == NOWALL ){printf(" %4x ",COST[wx-1][wy][1]);}
				}
		} 
		printf("   |\n");	
	} 
	printf("+");
	for (int wx = 0; wx < MAZESIZE_X; wx++) {printf("-----+"); }
	printf("\n");
	#endif

	printf("\n");
	return 0;
}
 
// **************************************************************************** 
// ダイクストラ
// 引数(goalx,goaly,goaldir)のノードから0,0,0のノードまでの経路を導出する
// 
// **************************************************************************** 
int dijkstra(int mask,char fast_check){
	unsigned short minn; 
	unsigned char vx,vy,vdir; //
	//unsigned short COST[N][N][2]; //グローバルに移動
	unsigned char USED[MAZESIZE_X][MAZESIZE_Y][2];//cost確定フラグ用 1bitでいい
	unsigned char tmpcnt; //ノード拡張用
	//unsigned short min_cost;

	/* 初期化 */
	for(int cx = 0; cx < MAZESIZE_X ; cx++){
		for(int cy = 0; cy < MAZESIZE_Y ; cy++){
			COST[cx][cy][0] = INF; //スタートからのcostを無限大で初期化
			USED[cx][cy][0] = FALSE;//未確定ノードのとして初期化
			COST[cx][cy][1] = INF; 
			USED[cx][cy][1] = FALSE; 
			index[cx][cy][0].x = cx;
			index[cx][cy][0].y = cy;
			index[cx][cy][0].dir = 0;
			index[cx][cy][1].x = cx;
			index[cx][cy][1].y = cy;
			index[cx][cy][1].dir = 1;
		}
	}
	COST[goalx][goaly][goaldir] = 0; // スタート地点は座標0,0 の北のノード
	
	//min_cost=INF;

	while(1){

		/* 未確定ノード(!USED[cx][cy][dir])の中から距離が最も小さいノードを選ぶ */
		minn = INF;
		//if(minn>min_cost+1){minn=min_cost+1;}
		for(int cx = 0; cx < MAZESIZE_X ; cx++){
			for(int cy = 0; cy < MAZESIZE_Y ; cy++){
	 			for(int dir=0 ;dir<2;dir++){
					if(USED[cx][cy][dir]==FALSE && minn > COST[cx][cy][dir] ) {
						minn = COST[cx][cy][dir];
						vx = cx;
						vy = cy;
						vdir = dir;
					}
	 			}
		 	}
		}
		USED[vx][vy][vdir] = TRUE; //このノードまでのcost(時間)は確定
		//printf("最小ﾉ-ﾄﾞ:%d,%d %d 値:%d \n",vx,vy,vdir,min);

		//if(minn == min_cost+1){
		if(minn == INF){
			//printf("goalまでの経路無し\n");
			return INF;
		}

		/* 最小ノードがスタート箇所のノードになった（これ以上の計算は不要） */
		if(vx == 0   && vy == 0  && vdir == 0){return COST[ vx][vy][vdir]; }//スタート位置のノード

		
		//min_cost = INF;

		/* 今確定したノードに直接つながっているノードに関して、今確定したノードを経由して到達する距離を計算して、より短い距離になれば更新する */
		if(vdir==0){//北or南向き
			if(index[vx][vy][0].y <= vy){//南方向から来た。迎える方向は 北、北東、北西
			if((wall[vx][vy+1].north & mask)== NOWALL && COST[vx  ][vy+1][0] > COST[vx][vy][0] + ex_straight[1] ){//北に進める
				COST[vx][vy+1][0] = COST[vx][vy][0] + ex_straight[1];
				//if(COST[vx][vy+1][0] <min_cost){min_cost=COST[vx][vy+1][0];}
				//printf("北に行ける %d \n",COST[vx][vy+1][0]);
				//最短ルート復元用に前のノードへのポインタをindexに格納する
				index[vx][vy+1][0].x = vx;
				index[vx][vy+1][0].y = vy;
				index[vx][vy+1][0].dir = 0;
				
				if(fast_check==EXACT_SEACH){
					//北方向にノードを拡張をする
					tmpcnt=2;
					while((wall[vx][vy+tmpcnt].north & mask)== NOWALL && COST[vx  ][vy+tmpcnt][0] > COST[vx][vy][0] + ex_straight[tmpcnt]){
						COST[vx][vy+tmpcnt][0] = COST[vx][vy][0] + ex_straight[tmpcnt];
						//printf("北にもっと行ける %d \n",COST[vx][vy+tmpcnt][0]);
						//最短ルート復元用に前のノードへのポインタをindexに格納する
						index[vx][vy+tmpcnt][0].x = vx;
						index[vx][vy+tmpcnt][0].y = vy;
						index[vx][vy+tmpcnt][0].dir = 0;
						tmpcnt++;
					}
				}

			}
			if((wall[vx][vy+1].east & mask) == NOWALL && COST[vx  ][vy+1][1] > COST[vx][vy][0] + ex_diagonalt[1] ){//北東に進める
				COST[vx][vy+1][1] = COST[vx][vy][0] + ex_diagonalt[1];
				//if(COST[vx][vy+1][1] <min_cost){min_cost=COST[vx][vy+1][1];}
				//printf("北東に行ける %d \n",COST[vx][vy+1][1]);
				index[vx][vy+1][1].x = vx;
				index[vx][vy+1][1].y = vy;
				index[vx][vy+1][1].dir = 0;

				if(fast_check==EXACT_SEACH){
					#ifdef EN_DIA
					//北東方向にノードを拡張をする
					tmpcnt=2;
					while( (tmpcnt%2==0 && (wall[vx+(tmpcnt/2)][vy+((tmpcnt+1)/2)].north & mask) == NOWALL && COST[vx+(tmpcnt/2)][vy+((tmpcnt+1)/2)][0] > COST[vx][vy][0] + ex_diagonalt[tmpcnt] ) ||
						   (tmpcnt%2==1 && wall[vx+(tmpcnt/2)][vy+((tmpcnt+1)/2)].east  == NOWALL && COST[vx+(tmpcnt/2)][vy+((tmpcnt+1)/2)][1] > COST[vx][vy][0] + ex_diagonalt[tmpcnt] ) ){ 
							if(tmpcnt%2==0){
								COST[vx+(tmpcnt/2)][vy+((tmpcnt+1)/2)][0] = COST[vx][vy][0] + ex_diagonalt[tmpcnt];
								//printf("北東にもっと行ける %d \n",COST[vx+(tmpcnt/2)][vy+((tmpcnt+1)/2)][0]);
								index[vx+(tmpcnt/2)][vy+((tmpcnt+1)/2)][0].x = vx;
								index[vx+(tmpcnt/2)][vy+((tmpcnt+1)/2)][0].y = vy;
								index[vx+(tmpcnt/2)][vy+((tmpcnt+1)/2)][0].dir = 0;
							}else{
								COST[vx+(tmpcnt/2)][vy+((tmpcnt+1)/2)][1] = COST[vx][vy][0] + ex_diagonalt[tmpcnt];
								//printf("北東にもっと行ける %d \n",COST[vx+(tmpcnt/2)][vy+((tmpcnt+1)/2)][1]);
								index[vx+(tmpcnt/2)][vy+((tmpcnt+1)/2)][1].x = vx;
								index[vx+(tmpcnt/2)][vy+((tmpcnt+1)/2)][1].y = vy;
								index[vx+(tmpcnt/2)][vy+((tmpcnt+1)/2)][1].dir = 0;
							}
						tmpcnt++;
					}
					#endif
				}
				   
			}
			if((wall[vx][vy+1].west & mask) == NOWALL && COST[vx-1][vy+1][1] > COST[vx][vy][0] + ex_diagonalt[1] ){//北西に進める
				COST[vx-1][vy+1][1] = COST[vx][vy][0] + ex_diagonalt[1];
				//if(COST[vx-1][vy+1][1] <min_cost){min_cost=COST[vx-1][vy+1][1];}
				//printf("北西に行ける %d \n",COST[vx-1][vy+1][1]);
				index[vx-1][vy+1][1].x = vx;
				index[vx-1][vy+1][1].y = vy;
				index[vx-1][vy+1][1].dir = 0;
				
				if(fast_check==EXACT_SEACH){
				#ifdef EN_DIA
				//北西方向にノードを拡張をする
				tmpcnt=2;
				while( (tmpcnt%2==0 && (wall[vx-((tmpcnt)/2)][vy+((tmpcnt+1)/2)].north & mask)== NOWALL && COST[vx-((tmpcnt+1)/2)][vy+((tmpcnt+1)/2)][0] > COST[vx][vy][0] + ex_diagonalt[tmpcnt] ) ||
					   (tmpcnt%2==1 && wall[vx-((tmpcnt)/2)][vy+((tmpcnt+1)/2)].west  == NOWALL && COST[vx-((tmpcnt+1)/2)][vy+((tmpcnt+1)/2)][1] > COST[vx][vy][0] + ex_diagonalt[tmpcnt] ) ){ 
						if(tmpcnt%2==0){
							COST[vx-((tmpcnt+1)/2)][vy+((tmpcnt+1)/2)][0] = COST[vx][vy][0] + ex_diagonalt[tmpcnt];
							//printf("北西にもっと行ける %d \n",COST[vx-(tmpcnt/2)][vy+((tmpcnt)/2)][0]);
							index[vx-((tmpcnt+1)/2)][vy+((tmpcnt+1)/2)][0].x = vx;
							index[vx-((tmpcnt+1)/2)][vy+((tmpcnt+1)/2)][0].y = vy;
							index[vx-((tmpcnt+1)/2)][vy+((tmpcnt+1)/2)][0].dir = 0;
						}else{
							COST[vx-((tmpcnt+1)/2)][vy+((tmpcnt+1)/2)][1] = COST[vx][vy][0] + ex_diagonalt[tmpcnt];
							//printf("北西にもっと行ける %d \n",COST[vx-((tmpcnt)/2)][vy+((tmpcnt+1)/2)][1]);
							index[vx-((tmpcnt+1)/2)][vy+((tmpcnt+1)/2)][1].x = vx;
							index[vx-((tmpcnt+1)/2)][vy+((tmpcnt+1)/2)][1].y = vy;
							index[vx-((tmpcnt+1)/2)][vy+((tmpcnt+1)/2)][1].dir = 0;
						}
					tmpcnt++;
				}
				#endif
				}

			}
			
			}
			if(index[vx][vy][0].y >= vy){//北方向から来た。迎える方向は 南、南東、南西
			if((wall[vx][vy  ].south & mask)== NOWALL && COST[vx  ][vy-1][0] > COST[vx][vy][0] + ex_straight[1] ){//南に進める
				COST[vx][vy-1][0] = COST[vx][vy][0] + ex_straight[1];
				//if(COST[vx][vy-1][0]<min_cost){min_cost=COST[vx][vy-1][0];}
				//printf("南に行ける\n");
				index[vx][vy-1][0].x = vx;
				index[vx][vy-1][0].y = vy;
				index[vx][vy-1][0].dir = 0;
				
				if(fast_check==EXACT_SEACH){
				//南方向にノードを拡張をする
				tmpcnt=2;
				while((wall[vx][vy-(tmpcnt-1)].south & mask)== NOWALL && COST[vx  ][vy-tmpcnt][0] > COST[vx][vy][0] + ex_straight[tmpcnt]){
					COST[vx][vy-tmpcnt][0] = COST[vx][vy][0] + ex_straight[tmpcnt];
					//printf("南にもっと行ける %d \n",COST[vx][vy-tmpcnt][0]);
					//最短ルート復元用に前のノードへのポインタをindexに格納する
					index[vx][vy-tmpcnt][0].x = vx;
					index[vx][vy-tmpcnt][0].y = vy;
					index[vx][vy-tmpcnt][0].dir = 0;
					tmpcnt++;
				}
				}
				
			}
			if((wall[vx][vy  ].east & mask) == NOWALL && COST[vx  ][vy  ][1] > COST[vx][vy][0] + ex_diagonalt[1]){//南東に進める
				COST[vx  ][vy][1] = COST[vx][vy][0] + ex_diagonalt[1];
				//if(COST[vx  ][vy][1] <min_cost){min_cost=COST[vx  ][vy][1] ;}
				//printf("南東に行ける\n");
				index[vx  ][vy][1].x = vx;
				index[vx  ][vy][1].y = vy;
				index[vx  ][vy][1].dir = 0;
				
				if(fast_check==EXACT_SEACH){
				#ifdef EN_DIA
				//南東方向にノードを拡張をする
				tmpcnt=2;
				while( (tmpcnt%2==0 && (wall[vx+(tmpcnt/2)][vy-(tmpcnt/2)].north & mask) == NOWALL && COST[vx+(tmpcnt/2)][vy-(tmpcnt/2)][0] > COST[vx][vy][0] + ex_diagonalt[tmpcnt] ) ||
					   (tmpcnt%2==1 && wall[vx+(tmpcnt/2)][vy-(tmpcnt/2)].east  == NOWALL && COST[vx+(tmpcnt/2)][vy-(tmpcnt/2)][1] > COST[vx][vy][0] + ex_diagonalt[tmpcnt] ) ){ 
						if(tmpcnt%2==0){
							COST[vx+(tmpcnt/2)][vy-(tmpcnt/2)][0] = COST[vx][vy][0] + ex_diagonalt[tmpcnt];
							//printf("南東にもっと行ける %d \n",COST[vx+(tmpcnt/2)][vy-(tmpcnt/2)][0]);
							index[vx+(tmpcnt/2)][vy-(tmpcnt/2)][0].x = vx;
							index[vx+(tmpcnt/2)][vy-(tmpcnt/2)][0].y = vy;
							index[vx+(tmpcnt/2)][vy-(tmpcnt/2)][0].dir = 0;
						}else{
							COST[vx+(tmpcnt/2)][vy-(tmpcnt/2)][1] = COST[vx][vy][0] + ex_diagonalt[tmpcnt];
							//printf("南東にもっと行ける %d \n",COST[vx+(tmpcnt/2)][vy-(tmpcnt/2)][1]);
							index[vx+(tmpcnt/2)][vy-(tmpcnt/2)][1].x = vx;
							index[vx+(tmpcnt/2)][vy-(tmpcnt/2)][1].y = vy;
							index[vx+(tmpcnt/2)][vy-(tmpcnt/2)][1].dir = 0;
						}
					tmpcnt++;
				}
				#endif
				}
				
			}
			if((wall[vx][vy  ].west & mask) == NOWALL && COST[vx-1][vy  ][1] > COST[vx][vy][0] + ex_diagonalt[1] ){//南西に進める
				COST[vx-1][vy][1] = COST[vx][vy][0] + ex_diagonalt[1];
				//if(COST[vx-1][vy][1] <min_cost){min_cost=COST[vx-1][vy][1];}
				//printf("南西に行ける\n");
				index[vx-1][vy][1].x = vx;
				index[vx-1][vy][1].y = vy;
				index[vx-1][vy][1].dir = 0;
				
				if(fast_check==EXACT_SEACH){
				#ifdef EN_DIA
				//南西方向にノードを拡張をする
				tmpcnt=2;
				while( (tmpcnt%2==0 && (wall[vx-((tmpcnt+1)/2)][vy-(tmpcnt/2)].north & mask)== NOWALL && COST[vx-((tmpcnt+1)/2)][vy-(tmpcnt/2)][0] > COST[vx][vy][0] + ex_diagonalt[tmpcnt] ) ||
					   (tmpcnt%2==1 && wall[vx-((tmpcnt+1)/2)][vy-(tmpcnt/2)].east  == NOWALL && COST[vx-((tmpcnt+1)/2)][vy-(tmpcnt/2)][1] > COST[vx][vy][0] + ex_diagonalt[tmpcnt] ) ){ 
						if(tmpcnt%2==0){
							COST[vx-((tmpcnt+1)/2)][vy-(tmpcnt/2)][0] = COST[vx][vy][0] + ex_diagonalt[tmpcnt];
							//printf("南西にもっと行ける %d \n",COST[vx-((tmpcnt+1)/2)][vy-(tmpcnt/2)][0]);
							index[vx-((tmpcnt+1)/2)][vy-(tmpcnt/2)][0].x = vx;
							index[vx-((tmpcnt+1)/2)][vy-(tmpcnt/2)][0].y = vy;
							index[vx-((tmpcnt+1)/2)][vy-(tmpcnt/2)][0].dir = 0;
						}else{
							COST[vx-((tmpcnt+1)/2)][vy-(tmpcnt/2)][1] = COST[vx][vy][0] + ex_diagonalt[tmpcnt];
							//printf("南西にもっと行ける %d \n",COST[vx-((tmpcnt+1)/2)][vy-(tmpcnt/2)][1]);
							index[vx-((tmpcnt+1)/2)][vy-(tmpcnt/2)][1].x = vx;
							index[vx-((tmpcnt+1)/2)][vy-(tmpcnt/2)][1].y = vy;
							index[vx-((tmpcnt+1)/2)][vy-(tmpcnt/2)][1].dir = 0;
						}
					tmpcnt++;
				}
				#endif
				}
			}
			}
			
		}else{
			if(index[vx][vy][1] .x <= vx){//西から東に来た 進める方向は東、北東、南東
			//東												/**    ここからY方向向き      **/
			if((wall[vx+1][vy].east & mask) == NOWALL && COST[vx+1][vy  ][1] > COST[vx][vy][1] + ex_straight[1] ){//東に進める
				COST[vx+1][vy][1] = COST[vx][vy][1] + ex_straight[1];
				//if(COST[vx+1][vy][1] <min_cost){min_cost=COST[vx+1][vy][1];}
				//printf("東に行ける %d\n",COST[vx+1][vy][1]);
				index[vx+1][vy][1] .x = vx;
				index[vx+1][vy][1] .y = vy;
				index[vx+1][vy][1] .dir = 1;
				
				if(fast_check==EXACT_SEACH){
				//東方向にノードを拡張をする
				tmpcnt=2;
				while((wall[vx+tmpcnt][vy].east & mask) == NOWALL && COST[vx+tmpcnt][vy  ][1] > COST[vx][vy][1] + ex_straight[tmpcnt]){
					COST[vx+tmpcnt][vy][1] = COST[vx][vy][1] + ex_straight[tmpcnt];
					//printf("東にもっと行ける %d\n",COST[vx+tmpcnt][vy][1]);
					//最短ルート復元用に前のノードへのポインタをindexに格納する
					index[vx+tmpcnt][vy][1] .x = vx;
					index[vx+tmpcnt][vy][1] .y = vy;
					index[vx+tmpcnt][vy][1] .dir = 1;
					tmpcnt++;
				}
				}
				
			}
			if((wall[vx+1][vy].north & mask)== NOWALL && COST[vx+1][vy  ][0] > COST[vx][vy][1] + ex_diagonalt[1] ){//北東に進める
				COST[vx+1][vy][0] = COST[vx][vy][1] + ex_diagonalt[1];
				//if(COST[vx+1][vy][0] <min_cost){min_cost=COST[vx+1][vy][0];}
				//printf("北東に行ける %d\n",COST[vx+1][vy][0]);
				index[vx+1][vy][0] .x = vx;
				index[vx+1][vy][0] .y = vy;
				index[vx+1][vy][0] .dir = 1;
				
				if(fast_check==EXACT_SEACH){
				#if 1
				//北東方向にノードを拡張をする
				tmpcnt=2;
				while( (tmpcnt%2==0 && (wall[vx+((tmpcnt+1)/2)][vy+((tmpcnt)/2)].east & mask) == NOWALL && COST[vx+((tmpcnt+1)/2)][vy+((tmpcnt)/2)][1] > COST[vx][vy][0] + ex_diagonalt[tmpcnt] ) ||
					   (tmpcnt%2==1 && wall[vx+((tmpcnt+1)/2)][vy+((tmpcnt)/2)].north == NOWALL && COST[vx+((tmpcnt+1)/2)][vy+((tmpcnt)/2)][0] > COST[vx][vy][0] + ex_diagonalt[tmpcnt] ) ){ 
						if(tmpcnt%2==0){
							COST[vx+((tmpcnt+1)/2)][vy+((tmpcnt)/2)][1] = COST[vx][vy][0] + ex_diagonalt[tmpcnt];
							//printf("北東にもっと行ける %d \n",COST[vx+((tmpcnt+1)/2)][vy+((tmpcnt)/2)][1]);
							index[vx+((tmpcnt+1)/2)][vy+((tmpcnt)/2)][1].x = vx;
							index[vx+((tmpcnt+1)/2)][vy+((tmpcnt)/2)][1].y = vy;
							index[vx+((tmpcnt+1)/2)][vy+((tmpcnt)/2)][1].dir = 1;
						}else{
							COST[vx+((tmpcnt+1)/2)][vy+((tmpcnt)/2)][0] = COST[vx][vy][0] + ex_diagonalt[tmpcnt];
							//printf("北東にもっと行ける %d \n",COST[vx+((tmpcnt+1)/2)][vy+((tmpcnt)/2)][0]);
							index[vx+((tmpcnt+1)/2)][vy+((tmpcnt)/2)][0].x = vx;
							index[vx+((tmpcnt+1)/2)][vy+((tmpcnt)/2)][0].y = vy;
							index[vx+((tmpcnt+1)/2)][vy+((tmpcnt)/2)][0].dir = 1;
						}
					tmpcnt++;
				}
				#endif
				}
				
			}
			if((wall[vx+1][vy].south & mask)== NOWALL && COST[vx+1][vy-1][0] > COST[vx][vy][1] + ex_diagonalt[1] ){//南東に進める
				COST[vx+1][vy-1][0] = COST[vx][vy][1] + ex_diagonalt[1];
				//if(COST[vx+1][vy-1][0] <min_cost){min_cost=COST[vx+1][vy-1][0];}
				//printf("南東に行ける %d\n",COST[vx+1][vy-1][0]);
				index[vx+1][vy-1][0] .x = vx;
				index[vx+1][vy-1][0] .y = vy;
				index[vx+1][vy-1][0] .dir = 1;
				
				if(fast_check==EXACT_SEACH){
				#if 1
				//南東方向にノードを拡張をする
				tmpcnt=2;
				while( (tmpcnt%2==0 && (wall[vx+(tmpcnt/2)][vy-(tmpcnt/2)].east & mask) == NOWALL && COST[vx+(tmpcnt/2)][vy-(tmpcnt/2)][0] > COST[vx][vy][0] + ex_diagonalt[tmpcnt] ) ||
					   (tmpcnt%2==1 && wall[vx+(tmpcnt/2)][vy-(tmpcnt/2)].north == NOWALL && COST[vx+(tmpcnt/2)][vy-(tmpcnt/2)][1] > COST[vx][vy][0] + ex_diagonalt[tmpcnt] ) ){ 
						if(tmpcnt%2==0){
							COST[vx+(tmpcnt/2)][vy-(tmpcnt/2)][1] = COST[vx][vy][0] + ex_diagonalt[tmpcnt];
							//printf("南東にもっと行ける %d \n",COST[vx+(tmpcnt/2)][vy-(tmpcnt/2)][1]);
							index[vx+(tmpcnt/2)][vy-(tmpcnt/2)][1].x = vx;
							index[vx+(tmpcnt/2)][vy-(tmpcnt/2)][1].y = vy;
							index[vx+(tmpcnt/2)][vy-(tmpcnt/2)][1].dir = 0;
						}else{
							COST[vx+(tmpcnt/2)][vy-(tmpcnt/2)][0] = COST[vx][vy][1] + ex_diagonalt[tmpcnt];
							//printf("南東にもっと行ける %d \n",COST[vx+(tmpcnt/2)][vy-(tmpcnt/2)][0]);
							index[vx+(tmpcnt/2)][vy-(tmpcnt/2)][0].x = vx;
							index[vx+(tmpcnt/2)][vy-(tmpcnt/2)][0].y = vy;
							index[vx+(tmpcnt/2)][vy-(tmpcnt/2)][0].dir = 0;
						}
					tmpcnt++;
				}
				#endif
				}
			}
			
			}
			if(index[vx][vy][1] .x >= vx){//東から西に来た 進める方向は西、北西、南西
				
			if((wall[vx  ][vy].west & mask) == NOWALL && COST[vx-1][vy  ][1] > COST[vx][vy][1] + ex_straight[1] ){//西に進める
				COST[vx-1][vy][1] = COST[vx][vy][1] + ex_straight[1];
				//if(COST[vx-1][vy][1] <min_cost){min_cost=COST[vx-1][vy][1];}
				//printf("西に行ける %d\n",COST[vx-1][vy][1]);
				index[vx-1][vy][1] .x = vx;
				index[vx-1][vy][1] .y = vy;
				index[vx-1][vy][1] .dir = 1;
				
				if(fast_check==EXACT_SEACH){
				//西方向にノードを拡張をする
				tmpcnt=2;
				while((wall[vx-(tmpcnt-1)][vy].west & mask) == NOWALL && COST[vx-tmpcnt][vy  ][1] > COST[vx][vy][1] + ex_straight[tmpcnt]){
					COST[vx-tmpcnt][vy][1] = COST[vx][vy][1] + ex_straight[tmpcnt];
					//printf("西にもっと行ける %d\n",COST[vx-tmpcnt][vy][1]);
					//最短ルート復元用に前のノードへのポインタをindexに格納する
					index[vx-tmpcnt][vy][1] .x = vx;
					index[vx-tmpcnt][vy][1] .y = vy;
					index[vx-tmpcnt][vy][1] .dir = 1;
					tmpcnt++;
				}
				}
				
			}
			if((wall[vx][vy].north & mask)== NOWALL && COST[vx  ][vy  ][0] > COST[vx][vy][1] + ex_diagonalt[1]){//北西に進める
				COST[vx][vy][0] = COST[vx][vy][1] + ex_diagonalt[1];
				//if(COST[vx][vy][0] <min_cost){min_cost=COST[vx][vy][0];}
				//printf("北西に行ける %d\n",COST[vx][vy][0]);
				index[vx][vy][0] .x = vx;
				index[vx][vy][0] .y = vy;
				index[vx][vy][0] .dir = 1;
				
				if(fast_check==EXACT_SEACH){
				#if 1
				//北西方向にノードを拡張をする OK
				tmpcnt=2;
				while( (tmpcnt%2==0 && (wall[vx-((tmpcnt)/2)][vy+((tmpcnt)/2)].east & mask) == NOWALL && COST[vx-((tmpcnt)/2)][vy+((tmpcnt)/2)][1] > COST[vx][vy][1] + ex_diagonalt[tmpcnt] ) ||
					   (tmpcnt%2==1 && wall[vx-((tmpcnt)/2)][vy+((tmpcnt)/2)].north == NOWALL && COST[vx-((tmpcnt)/2)][vy+((tmpcnt)/2)][0] > COST[vx][vy][1] + ex_diagonalt[tmpcnt] ) ){ 
						if(tmpcnt%2==0){
							COST[vx-((tmpcnt)/2)][vy+((tmpcnt)/2)][1] = COST[vx][vy][1] + ex_diagonalt[tmpcnt];
							//printf("北西にもっと行ける %d \n",COST[vx-((tmpcnt)/2)][vy+((tmpcnt)/2)][1]);
							index[vx-((tmpcnt)/2)][vy+((tmpcnt)/2)][1].x = vx;
							index[vx-((tmpcnt)/2)][vy+((tmpcnt)/2)][1].y = vy;
							index[vx-((tmpcnt)/2)][vy+((tmpcnt)/2)][1].dir = 1;
						}else{
							COST[vx-((tmpcnt)/2)][vy+((tmpcnt)/2)][0] = COST[vx][vy][1] + ex_diagonalt[tmpcnt];
							//printf("北西にもっと行ける %d \n",COST[vx-((tmpcnt)/2)][vy+((tmpcnt)/2)][0]);
							index[vx-((tmpcnt)/2)][vy+((tmpcnt)/2)][0].x = vx;
							index[vx-((tmpcnt)/2)][vy+((tmpcnt)/2)][0].y = vy;
							index[vx-((tmpcnt)/2)][vy+((tmpcnt)/2)][0].dir = 1;
						}
					tmpcnt++;
				}
				#endif
				}
				
			}
			if((wall[vx  ][vy].south & mask)== NOWALL && COST[vx  ][vy-1][0] > COST[vx][vy][1] + ex_diagonalt[1] ){//南西に進める
				COST[vx][vy-1][0] = COST[vx][vy][1] + ex_diagonalt[1];
				//if(COST[vx][vy-1][0] <min_cost){COST[vx][vy-1][0];}
				//printf("南西に行ける %d\n",COST[vx][vy-1][0]);
				index[vx][vy-1][0] .x = vx;
				index[vx][vy-1][0] .y = vy;
				index[vx][vy-1][0] .dir = 1;
				
				if(fast_check==EXACT_SEACH){
				#if 1
				//南西方向にノードを拡張をする
				tmpcnt=2;
				while( (tmpcnt%2==0 && (wall[vx-((tmpcnt)/2)][vy-((tmpcnt+1)/2)].east & mask) == NOWALL && COST[vx-((tmpcnt)/2)][vy-((tmpcnt+1)/2)][0] > COST[vx][vy][0] + ex_diagonalt[tmpcnt] ) ||
					   (tmpcnt%2==1 && wall[vx-((tmpcnt)/2)][vy-((tmpcnt+1)/2)].north == NOWALL && COST[vx-((tmpcnt)/2)][vy-((tmpcnt+1)/2)][1] > COST[vx][vy][0] + ex_diagonalt[tmpcnt] ) ){ 
						if(tmpcnt%2==0){
							COST[vx-((tmpcnt)/2)][vy-((tmpcnt+1)/2)][1] = COST[vx][vy][0] + ex_diagonalt[tmpcnt];
							//printf("南西にもっと行ける %d \n",COST[vx-((tmpcnt)/2)][vy-((tmpcnt+1)/2)][1]);
							index[vx-((tmpcnt)/2)][vy-((tmpcnt+1)/2)][1].x = vx;
							index[vx-((tmpcnt)/2)][vy-((tmpcnt+1)/2)][1].y = vy;
							index[vx-((tmpcnt)/2)][vy-((tmpcnt+1)/2)][1].dir = 0;
						}else{
							COST[vx-((tmpcnt)/2)][vy-((tmpcnt+1)/2)][0] = COST[vx][vy][0] + ex_diagonalt[tmpcnt];
							//printf("南西にもっと行ける %d \n",COST[vx-((tmpcnt)/2)][vy-((tmpcnt+1)/2)][0]);
							index[vx-((tmpcnt)/2)][vy-((tmpcnt+1)/2)][0].x = vx;
							index[vx-((tmpcnt)/2)][vy-((tmpcnt+1)/2)][0].y = vy;
							index[vx-((tmpcnt)/2)][vy-((tmpcnt+1)/2)][0].dir = 0;
						}
					tmpcnt++;
				}
				#endif
				}
				
			}
			}
		}

	}
  
}


 // **************************************************************************** 
 // 迷路と導出した経路とを表示する
 // 
 // **************************************************************************** 
void showmap(void){
	short nodex,nodey,nodedir;
	short tmpx,tmpy,tmpdir;
	short cnt_loc,flugloc;
	//int cntex;
	short tmpi;
	
	/* 経路を表示(区画) */
	printf("start(0,0)からgoal(%d,%d)までの経路(区間)\n",goalx,goaly);
	nodex = 0;
	nodey = 0;
	nodedir = 0;
	#ifdef EN_PRINT_1
		printf("%d,%d", nodex,nodey);
	#endif
	location[0].x=0;
	location[0].y=0;
	cnt_loc=1;
	
	while(1){
		tmpx=nodex;
		tmpy=nodey;
		tmpdir=nodedir;
		nodex = index[tmpx][tmpy][tmpdir].x;
		nodey = index[tmpx][tmpy][tmpdir].y;
		nodedir = index[tmpx][tmpy][tmpdir].dir;
	
		
		
		if(nodey!=tmpy && nodex!=tmpx){
			if( (nodex-tmpx) >0 && (nodey-tmpy)>0 ){//北東方向
			while(nodey!=tmpy && nodex!=tmpx ){
				if( (nodex-tmpx) == (nodey-tmpy) ){
					
					location[cnt_loc].x=tmpx;
					if(nodey-tmpy>0){location[cnt_loc].y=tmpy+1;tmpy++;}else{location[cnt_loc].y=tmpy-1;tmpy--;}
				}else{
					if(nodex-tmpx>0){location[cnt_loc].x=tmpx+1;tmpx++;}else{location[cnt_loc].x=tmpx-1;tmpx--;}
					location[cnt_loc].y=tmpy;
				}
				#ifdef EN_PRINT_1
					printf("->北東にななめ補完:%d,%d",location[cnt_loc].x ,location[cnt_loc].y);
				#endif
				cnt_loc++;
			}
			}else if( (nodex-tmpx) <0 && (nodey-tmpy)<0 ){//南西方向
			while(nodey!=tmpy && nodex!=tmpx ){
				if( ( (nodex-tmpx) == (nodey-tmpy)&&nodedir==0 ) || ( (nodex-tmpx) != (nodey-tmpy)&&nodedir==1 ) ){
					if(nodex-tmpx>0){location[cnt_loc].x=tmpx+1;tmpx++;}else{location[cnt_loc].x=tmpx-1;tmpx--;}
					location[cnt_loc].y=tmpy;
				}else{	
					location[cnt_loc].x=tmpx;
					if(nodey-tmpy>0){location[cnt_loc].y=tmpy+1;tmpy++;}else{location[cnt_loc].y=tmpy-1;tmpy--;}
				}
				#ifdef EN_PRINT_1
					printf("->南西にななめ補完:%d,%d",location[cnt_loc].x ,location[cnt_loc].y);
				#endif
				cnt_loc++;
			}
			}else if((nodex-tmpx)>0 && (nodey-tmpy)<0){//南東方向
			while(nodey!=tmpy && nodex!=tmpx ){
				if(  (tmpx-nodex) == (nodey-tmpy)  ){
					if(nodex-tmpx>0){location[cnt_loc].x=tmpx+1;tmpx++;}else{location[cnt_loc].x=tmpx-1;tmpx--;}
					location[cnt_loc].y=tmpy;
				}else{
					location[cnt_loc].x=tmpx;
					if(nodey-tmpy>0){location[cnt_loc].y=tmpy+1;tmpy++;}else{location[cnt_loc].y=tmpy-1;tmpy--;}
				}
				#ifdef EN_PRINT_1
					printf("->南東にななめ補完:%d,%d",location[cnt_loc].x ,location[cnt_loc].y);
				#endif
				cnt_loc++;
			}
			}else if((nodex-tmpx)<0 && (nodey-tmpy)>0){//北西方向
			while(nodey!=tmpy && nodex!=tmpx ){
				if( (tmpx-nodex) == (nodey-tmpy) ){
					location[cnt_loc].x=tmpx;
					if(nodey-tmpy>0){location[cnt_loc].y=tmpy+1;tmpy++;}else{location[cnt_loc].y=tmpy-1;tmpy--;}
				}else{
					if(nodex-tmpx>0){location[cnt_loc].x=tmpx+1;tmpx++;}else{location[cnt_loc].x=tmpx-1;tmpx--;}
					location[cnt_loc].y=tmpy;					
				}
				#ifdef EN_PRINT_1
					printf("->北西にななめ補完:%d,%d",location[cnt_loc].x ,location[cnt_loc].y);
				#endif
				cnt_loc++;
			}
			}
		}		
		//if(tmpx==nodex-1 && tmpy==nodey+1){printf(" -> %d,%d", nodex,nodey+1);location[cnt_loc].x=nodex;location[cnt_loc].y=(nodey+1);cnt_loc++;}
		//if(tmpx==nodex+1 && tmpy==nodey-1){printf(" -> %d,%d", nodex+1,nodey);location[cnt_loc].x=(nodex+1);location[cnt_loc].y=nodey;cnt_loc++;}
		
		if(tmpx==nodex && nodey>tmpy+1){//北側に埋める
			while(nodey>tmpy){
				location[cnt_loc].x=nodex;
				location[cnt_loc].y=tmpy+1;
				cnt_loc++;
				tmpy++;
			}
		}
		if(tmpx==nodex && nodey<tmpy-1){//南側に埋める
			while(nodey<tmpy){
				location[cnt_loc].x=nodex;
				location[cnt_loc].y=tmpy-1;;
				cnt_loc++;
				tmpy--;
			}
		}
		if(tmpy==nodey && nodex>tmpx+1){//東側に埋める
			while(nodex>tmpx){
				location[cnt_loc].x=tmpx+1;
				location[cnt_loc].y=nodey;
				cnt_loc++;
				tmpx++;
			}
		}
		if(tmpy==nodey && nodex<tmpx-1){//西側に埋める
			while(nodex<tmpx){
				location[cnt_loc].x=tmpx-1;;
				location[cnt_loc].y=nodey;
				cnt_loc++;
				tmpx--;
			}
		}
		
		
		if(!(tmpx==nodex && tmpy==nodey)){
			#ifdef EN_PRINT_1
				printf(" -> %d,%d", nodex,nodey);
			#endif
			location[cnt_loc].x=nodex;
			location[cnt_loc].y=nodey;
			cnt_loc++;
		}
		
		if (nodex == goalx && nodey==goaly && nodedir==goaldir){break;}
	}
	//printf("\n");



	//迷路上にプロット
	for(int wx=0;wx<MAZESIZE_X;wx++){
		for(int wy=0;wy<MAZESIZE_Y;wy++){
			map[wx][wy]=0xfff;
		}
	}
	cnt_loc--;
	for(int tloc=0; tloc<=cnt_loc; tloc++){
		map[location[tloc].x][location[tloc].y]=cnt_loc-tloc;
	}

	
	//ここから表示
	for(int wy=(MAZESIZE_Y-1);wy>=0 ;wy--){
		for(int wx=0 ;wx<=(MAZESIZE_X-1);wx++){	
			if(wall[wx  ][wy  ].north == WALL   ){printf("+---");}
			if(wall[wx  ][wy  ].north == NOWALL ){printf("+   ");}
		} 
		printf("+\n");
		for(int wx=0 ;wx<=(MAZESIZE_X-1);wx++){
			
			tmpi=map[wx][wy];//
			
			if(tmpi==0xfff){
				if(wall[wx  ][wy  ].west  == WALL   ){printf("|   ");}
				if(wall[wx  ][wy  ].west  == NOWALL ){printf("    ");}
			}else{
				if(wall[wx  ][wy  ].west  == WALL   ){printf("|%3x",tmpi);}
				if(wall[wx  ][wy  ].west  == NOWALL ){printf(" %3x",tmpi);}
			}
		} 
		printf("|\n");	
	} 
	printf("+");
	for (int wx = 0; wx < MAZESIZE_X; wx++) {printf("---+"); }
	printf("\n");
	printf("\n");
}

 // **************************************************************************** 
 // 迷路の壁情報を読み込む
 // 迷路情報はコチラから https://longmouse.web.fc2.com/zappin/mazedata/index.html
 // **************************************************************************** 
void setwall(void){
	unsigned char tmp;
	short mcfcnt=0;
	
	//迷路の壁情報
  // from https://longmouse.web.fc2.com/zappin/mazedata/index.html
	#if 1
	//MC2015AllJapanEFinal
	unsigned char mcf[MAZESIZE_X*MAZESIZE_Y]={14,12,5,5,5,6,13,4,4,7,12,5,6,12,5,6,9,2,12,5,6,9,4,3,9,4,3,14,9,3,12,3,12,3,10,12,0,6,11,12,6,11,12,0,6,13,0,7,10,13,0,2,11,9,4,3,9,4,3,11,8,6,9,6,10,13,2,9,6,12,1,6,12,1,6,12,3,8,7,10,10,14,9,6,9,0,7,8,2,13,0,3,12,3,12,3,10,8,6,9,6,11,12,3,9,6,11,12,3,13,0,7,10,10,10,14,9,6,9,4,4,3,12,3,12,6,9,6,10,10,10,8,6,9,6,9,3,12,3,12,3,9,4,3,10,10,10,10,8,7,9,6,12,3,12,3,14,13,0,7,10,10,10,10,8,5,7,8,2,13,2,12,1,6,11,14,10,10,10,10,8,7,12,3,9,6,9,0,7,8,7,10,10,10,10,10,10,12,1,5,5,1,6,11,12,3,12,2,10,10,9,3,9,1,5,5,5,5,1,5,3,12,3,10,10,9,5,5,5,5,5,5,5,5,5,5,5,1,7,10,9,5,5,5,5,5,5,5,5,5,5,5,5,5,5,3};
	goalx=8;
	goaly=7;
	goaldir=0;
	#endif
	

	
	//迷路フォーマットの変換
	for(int cy=0;cy<MAZESIZE_Y;cy++){
		for(int cx=0;cx<MAZESIZE_X;cx++){
			tmp = mcf[mcfcnt];
			mcfcnt++;
			wall[cx][cy].north = NOWALL;
			wall[cx][cy].east  = NOWALL;
			wall[cx][cy].south = NOWALL;
			wall[cx][cy].west  = NOWALL;
			if((tmp&0x01)==0x01){wall[cx][cy].north = WALL;}
			if((tmp&0x02)==0x02){wall[cx][cy].east = WALL;}
			if((tmp&0x04)==0x04){wall[cx][cy].south = WALL;}
			if((tmp&0x08)==0x08){wall[cx][cy].west = WALL;}
		}
	}

}
  
