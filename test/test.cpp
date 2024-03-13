//测试堆内存和栈内存的区别
#include <iostream>
#include <vector>
#include <string>

using namespace std;
int counter = 0;

void swap(vector<int>& nums, int a, int b) {
	int num = nums[a];
	nums[a] = nums[b];
	nums[b] = num;
}

void printArray(const vector<int>& arr) {
	cout << counter;
	cout << " -- Sorted array: ";
	counter++;
	for (int num : arr) {
		cout << num << " ";
	}
	cout << endl;
}

void quickSort(vector<int>& nums, int l, int r) {
	if (l >= r) return;
	int l_org = l;
	int r_org = r;
	int flag = l++;
	while (l <= r) {
		if (flag < l) {
			if (nums[r] < nums[flag]) {
				swap(nums, r, flag);
				flag = r;
			}
			r--;
		}
		else {
			if (nums[l] > nums[flag]) {
				swap(nums, l, flag);
				flag = l;
			}
			l++;
		}
	}
	printArray(nums);
	quickSort(nums, l_org, flag);
	quickSort(nums, flag+1, r_org);
}


struct ListNode {
	int val;
	ListNode* next;
	ListNode() : val(0), next(nullptr) {}
	ListNode(int x) : val(x), next(nullptr) {}
	ListNode(int x, ListNode* next) : val(x), next(next) {}
};

ListNode* addTwoNumbers(ListNode* l1, ListNode* l2) {
	ListNode* sum;
	ListNode* head = sum;
	int flag = 0;
	while (l1 != nullptr && l2 != nullptr) {
		int subSum = l1->val + l2->val + flag;
		if (subSum >= 10) {
			flag = 1;
			sum->val = subSum - 10;
		}
		else {
			flag = 0;
			sum->val = subSum;
		}
		l1 = l1->next;
		l2 = l2->next;
		sum->next = new ListNode();
		sum = sum->next;
	}
	while (l1 != nullptr) {
		sum->val = l1->val;
		sum->next = new ListNode();
		sum = sum->next;
		l1 = l1->next;
	}
	while (l2 != nullptr) {
		sum->val = l2->val;
		sum->next = new ListNode();
		sum = sum->next;
		l2 = l2->next;
	}
	return head;
}
int main()
{
	string s = "";
	Node* n = new Node();
	Node* n1 = new Node();
	n->next = n1;
	s += "1";
	s += "2";
	cout << stoi(s);
	vector<int> arr = { 9, 5, 1, 8, 3, 2, 7, 6, 4 };
	//cout << "Original array: ";
	//printArray(arr);
	//quickSort(arr, 0, arr.size() - 1);
	return 0;
}

