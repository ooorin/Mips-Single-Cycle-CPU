#include <cstdio>
#include <iostream>
#include <string>

using namespace std;

int main()
{
	freopen("bin.txt", "r", stdin);
	freopen("hex.txt", "w", stdout);
	string s;
	string ss;
	char c;
	int tmp;
	while(scanf("%c", &c) != EOF) {
		if (c == '\n')
		{
		for (int i = 0; i <= 28; i += 4) {
			tmp = 0;
			for (int j = 0; j < 4; j ++) {
				tmp = tmp *2 + int(s[i + j] - '0');
			}
			if (tmp == 10) cout << "A";
			else if (tmp == 11) cout << "B";
			else if (tmp == 12) cout << "C";
			else if (tmp == 13) cout << "D";
			else if (tmp == 14) cout << "E";
			else if (tmp == 15) cout << "F";
			else cout << tmp;
		}
			cout << ' ' << ss;
			cout << endl;
			s.clear();
			ss.clear();
		}
		else if (c == '0' || c == '1') s = s + c;
		else if (c != ' ') ss = ss + c;
		else continue;
	}
}
