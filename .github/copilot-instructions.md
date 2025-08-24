# GitHub Copilot Code Generation Instructions

1. **Prioritize Minimal Impact**  
   Understand the code's architectural context (dependencies, assumptions, history) before modification. Aim for the smallest change that fulfills the requirement while preserving existing functionality and patterns. Avoid unnecessary refactoring.

2. **Targeted Implementation**  
   Identify and modify only the essential code sections. Preserve unrelated code and maintain existing system behavior.

3. **Graduated Change Strategy**  
   - **Default:** Implement the minimal, focused change for the specific request.  
   - **If Necessary:** Offer moderate, localized refactoring.  
   - **Only if Explicitly Requested:** Perform comprehensive restructuring.

4. **Clarify Ambiguity**  
   If the required scope is unclear, request clarification before proceeding. Do not assume a broader scope than specified.

5. **Document Potential Enhancements**  
   Note related improvements outside the immediate scope without implementing them (e.g., “Function Y uses a similar pattern and could benefit from this update later.”).

6. **Ensure Reversibility**  
   Design changes to be easily revertible if they don't yield the intended outcome or introduce issues.

7. **Adhere to Code Quality Standards**  
   - **Clarity & Readability:** Use descriptive names; keep functions concise and single-purpose; follow style guides (e.g., PEP 8, Prettier).  
   - **Consistency:** Follow existing project patterns, conventions, and technology choices unless a change is justified.  
   - **Robust Error Handling:** Anticipate failures (I/O, network, input); use appropriate mechanisms (try-catch, specific exceptions); provide informative error messages.  
   - **Security:** Sanitize inputs; manage secrets securely (env vars/config tools); vet external libraries.  
   - **Testability:** Design for testability (e.g., dependency injection); ensure adequate test coverage.  
   - **Documentation:** Comment complex/non-obvious code; use standard formats (JSDoc, DocStrings).  
   - **Modern Implementation:** Check for and use modern, actively maintained APIs and libraries. Avoid deprecated or legacy code unless strictly necessary, and document any unavoidable use.

8. **Conventional Commit Messages**  
   Generate commit messages following the Conventional Commits specification (e.g., `feat(api): add user endpoint`). Use imperative mood. Infer type (`feat`, `fix`, `chore`, `refactor`, `test`, `docs`) and scope from the changes.