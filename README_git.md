# git使用与多人写作指南
全向轮已经实现了git本地与云端管理，建议在增加新功能或调试bug完成后，定期git commit记录更改内容。
## 本地git使用流程
### 如何提交修改
cd ~/zxzzb/src
git add . 将所有更改内容添加至暂存区
git status 查看暂存区内容
git commit -m "describe your change of this commit" 提交所有暂存区内容并简述本次修改内容 建议结尾注明更改者 例如 git commit -m "test commit --jerry"
git push -u origin OmniWheels 将本地分支提交到github的OmniWheels分支
### 如何回退版本 （待测试）
git log 查看所有提交历史 
git reset --hard <版本号> 版本号通过git log查看
## 多人协作流程
github仓库中 master分支存储了最稳定版本的代码
每辆车本地管理一个master分支 对应github云端的一个<车名>分支 eg. 全向轮本地master分支对应github云端OmniWheels分支
github管理猿可以定期合并仓库中的分支更新master分支（考虑到每辆车可能参数不同 也可以只停留在分支部分不进行合并）