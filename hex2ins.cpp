#include <iostream>
#include <cstdio>
#include <string>

using namespace std;

int main()
{
	freopen("hex.txt", "r", stdin);
	freopen("ins.txt", "w", stdout);
	
	string s;
	
	while(cin >> s) {
		string ans1;
		for (int i = 0; i < s.size(); i ++) {
			string bin;
			int tmp;
			if (s[i] >= '0' && s[i] <= '9')
				tmp = int(s[i] - '0');
			else
				tmp = int(s[i] - 'A' + 10);
			for (int j = 0; j < 4; j ++) {
				bin = bin + char(tmp % 2 + '0');
				tmp /= 2;
			}
			for (int j = 3; j >= 0; j --)
				ans1 = ans1 + bin[j];
		}	
		string ans;
		ans.clear();
		for (int i = 31; i >= 0; i --)
			ans += ans1[i];	

		string op;
		string func;
		string ins;
		int rtype;
		for (int i = 0; i < 6; i ++)
			op += ans[31 - i];
		for (int i = 0; i < 6; i ++)
			func += ans[5 - i];
		if (op == "000000") {
			rtype = 1;
			if (func == "100000") ins = "ADD";
			else if (func == "100010") ins = "SUB";
			else if (func == "101010") ins = "SLT";
			else ins = "ERROR"; 
		}
		else {
			rtype = 0;
			if (op == "100011") ins = "LW";
			else if (op == "101011") ins = "SW";
			else if (op == "000100") ins = "BEQ";
			else if (op == "000101") ins = "BNE";
			else if (op == "001000") ins = "ADDI";
			else ins = "ERROR"; 
		}
		
		int ns = 0, nt = 0, nd = 0;
		ns = 16 * int (ans[25] - '0') +
			 8 * int (ans[24] - '0') +
			 4 * int (ans[23] - '0') +
			 2 * int (ans[22] - '0') +
			 1 * int (ans[21] - '0');
		nt = 16 * int (ans[20] - '0') +
			 8 * int (ans[19] - '0') +
			 4 * int (ans[18] - '0') +
			 2 * int (ans[17] - '0') +
			 1 * int (ans[16] - '0');
		nd = 16 * int (ans[15] - '0') +
			 8 * int (ans[14] - '0') +
			 4 * int (ans[13] - '0') +
			 2 * int (ans[12] - '0') +
			 1 * int (ans[11] - '0');
			 
		string rs, rt, rd;
		rs = '$' + char(ns + '0');
		rt = '$' + char(nt + '0');
		rd = '$' + char(nd + '0');
		
		int imm = 0;
		string imm_s;
		imm_s.clear();
		for (int i = 0; i < 4; i ++)
			imm_s += s[7 - i];
		imm = 0;
		for (int i = 3; i >= 0; i --)
			if (imm_s[i] >= '0' && imm_s[i] <= '9')
				imm = imm * 16 + int(imm_s[i] - '0');
			else if (imm_s[i] >= 'A' && imm_s[i] <= 'F')
				imm = imm * 16 + int(imm_s[i] - 'A' + 10);
		
		if (rtype) {
			cout << s;
			cout << " # " << ins + " " << "$" << ns << " " << "$" << nt << " " << "$" << nd << " ";
		}
		else{
			cout << s;
			cout << " # " << ins + " " << "$" << ns << " " << "$" << nt << " " << imm << " ";
		}
		cout << endl;
	}
}
