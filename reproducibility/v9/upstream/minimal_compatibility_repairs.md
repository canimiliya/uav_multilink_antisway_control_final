# Upstream Compatibility Repairs

The pinned author source is retained outside the project tree at commit
e2ababf519e7b7cca8e23f42dcb8c34fae927037. Two execution-only repairs were
needed on Windows with a Blackwell-compatible PyTorch build.

1. validation.py loads lifting_func.pth, A.pth, and B.pth but never uses them.
   The published lifting_func.pth is a 689-byte truncated PyTorch archive. The
   complete lls_wrapper.pth loads correctly and is the object used by every
   original validation prediction. The compatibility run skipped only the
   three unused loads.
2. train.py used SummaryWriter.add_scalars, whose child-writer path mixed
   separators on Windows. It was replaced by two add_scalar calls with the same
   tags and values. Forward/backward computation, LLS least squares, optimizer,
   losses, and serialization were unchanged.

These repairs belong to the upstream smoke environment. They are not copied
into the V9 adaptation and do not modify any benchmark controller.
