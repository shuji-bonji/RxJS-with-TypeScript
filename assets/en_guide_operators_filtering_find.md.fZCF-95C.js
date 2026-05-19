import{_ as a,o as i,c as n,a2 as p}from"./chunks/framework.B0tZAgFO.js";const c=JSON.parse('{"title":"find - find the first value that satisfies the condition","description":"find is an RxJS filtering operator that finds the first value that satisfies a condition and outputs it, immediately completing the stream. It is ideal for user searches, inventory checks, error log detection, and other situations where you want to find a specific element in an array or list. If no value is found, it outputs undefined and in TypeScript the return value will be of type T | undefined.","frontmatter":{"description":"find is an RxJS filtering operator that finds the first value that satisfies a condition and outputs it, immediately completing the stream. It is ideal for user searches, inventory checks, error log detection, and other situations where you want to find a specific element in an array or list. If no value is found, it outputs undefined and in TypeScript the return value will be of type T | undefined."},"headers":[],"relativePath":"en/guide/operators/filtering/find.md","filePath":"en/guide/operators/filtering/find.md","lastUpdated":1779130692000}'),t={name:"en/guide/operators/filtering/find.md"};function e(l,s,h,k,r,d){return i(),n("div",null,[...s[0]||(s[0]=[p(`<h1 id="find-find-the-first-value-that-satisfies-the-condition" tabindex="-1">find - find the first value that satisfies the condition <a class="header-anchor" href="#find-find-the-first-value-that-satisfies-the-condition" aria-label="Permalink to &quot;find - find the first value that satisfies the condition&quot;">​</a></h1><p>The <code>find</code> operator finds the <strong>first value that satisfies the condition</strong> and outputs it, immediately completing the stream. If no value is found, it outputs <code>undefined</code>.</p><h2 id="🔰-basic-syntax-and-usage" tabindex="-1">🔰 Basic Syntax and Usage <a class="header-anchor" href="#🔰-basic-syntax-and-usage" aria-label="Permalink to &quot;🔰 Basic Syntax and Usage&quot;">​</a></h2><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { from } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { find } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> numbers$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> from</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">([</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">1</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">3</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">5</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">7</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">8</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">9</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">10</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">]);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">numbers$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  find</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">n</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> n </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">%</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 2</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> ===</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 0</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Output: 8(first even number)</span></span></code></pre></div><p><strong>Flow of operation</strong>: 1.</p><ol><li>check 1, 3, 5, 7 → condition not met</li><li>check 8 → condition satisfied → output 8 and complete</li><li>9, 10 are not evaluated</li></ol><p><a href="https://rxjs.dev/api/operators/find" target="_blank" rel="noreferrer">🌐 RxJS Official Documentation - <code>find</code></a></p><h2 id="🆚-contrast-with-first" tabindex="-1">🆚 Contrast with first <a class="header-anchor" href="#🆚-contrast-with-first" aria-label="Permalink to &quot;🆚 Contrast with first&quot;">​</a></h2><p><code>find</code> and <code>first</code> are similar, but their usage is different.</p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { from } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { find, first } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> numbers$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> from</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">([</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">1</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">3</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">5</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">7</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">8</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">9</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">10</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">]);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// first: First value satisfying the condition (condition is optional)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">numbers$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  first</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">n</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> n </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">&gt;</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 5</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Output: 7</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// find: First value satisfying the condition (condition is mandatory)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">numbers$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  find</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">n</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> n </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">&gt;</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 5</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Output: 7</span></span></code></pre></div><table tabindex="0"><thead><tr><th>Operator</th><th>Condition specification</th><th>If no value is found</th><th>Use case</th></tr></thead><tbody><tr><td><code>first()</code> option</td><td>Option</td><td>Error (<code>EmptyError</code>)</td><td>Get the first value.</td></tr><tr><td><code>first(predicate)</code> (default: <code>first(predicate)</code>)</td><td>Optional</td><td>Error (<code>EmptyError</code>)</td><td>Conditional get.</td></tr><tr><td>find(predicate)\`</td><td>required</td><td>Output <code>undefined</code>.</td><td>Search and existence check</td></tr></tbody></table><h2 id="💡-typical-utilization-pattern" tabindex="-1">💡 Typical utilization pattern <a class="header-anchor" href="#💡-typical-utilization-pattern" aria-label="Permalink to &quot;💡 Typical utilization pattern&quot;">​</a></h2><ol><li><strong>User Search</strong>.</li></ol><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">   import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { from } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">   import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { find } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">   interface</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> User</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">     id</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">:</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> number</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">     name</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">:</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> string</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">     email</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">:</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> string</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   }</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">   const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> users$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> from</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">([</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">     { id: </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">1</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, name: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Alice&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, email: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;alice@example.com&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> },</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">     { id: </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">2</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, name: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Bob&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, email: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;bob@example.com&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> },</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">     { id: </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">3</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, name: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Charlie&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, email: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;charlie@example.com&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> }</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   ] </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">as</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> User</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">[]);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">   // IDis2Search for users with</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   users$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">     find</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">user</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> user.id </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">===</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 2</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   ).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">user</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">     if</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> (user) {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">       console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`Found: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">user</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">name</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">     } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">else</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">       console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;User not found&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">     }</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   });</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">   // Output: Found: Bob</span></span></code></pre></div><ol start="2"><li><p><strong>Check inventory</strong></p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { from } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { find } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">interface</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> Product</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">  id</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">:</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> string</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">  name</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">:</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> string</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">  stock</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">:</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> number</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">}</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> products$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> from</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">([</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  { id: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;A1&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, name: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;NotebookPC&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, stock: </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">0</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> },</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  { id: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;A2&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, name: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Mouse&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, stock: </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">15</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> },</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  { id: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;A3&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, name: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Keyboard&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, stock: </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">8</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> }</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">] </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">as</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> Product</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">[]);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Find out what&#39;s out of stock</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">products$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  find</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">product</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> product.stock </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">===</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 0</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">product</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">  if</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> (product) {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">    console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`Out of Stock: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">product</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">name</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">else</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">    console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;All in stock&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  }</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">});</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Output: Out of Stock: NotebookPC</span></span></code></pre></div></li><li><p><strong>Search error log</strong></p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { from } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { find } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">interface</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> LogEntry</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">  timestamp</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">:</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> number</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">  level</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">:</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;info&#39;</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> |</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;warn&#39;</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> |</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;error&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">  message</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">:</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> string</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">}</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> logs$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> from</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">([</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  { timestamp: </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">1</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, level: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;info&#39;</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> as</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> const</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, message: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;App started&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> },</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  { timestamp: </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">2</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, level: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;info&#39;</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> as</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> const</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, message: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;User logged in&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> },</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  { timestamp: </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">3</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, level: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;error&#39;</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> as</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> const</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, message: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Connection failed&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> },</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  { timestamp: </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">4</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, level: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;info&#39;</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> as</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> const</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, message: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Retry successful&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> }</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">] </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">as</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> LogEntry</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">[]);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Find first error</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">logs$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  find</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">log</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> log.level </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">===</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;error&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">log</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">  if</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> (log) {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">    console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`Error Detection: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">log</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">message</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">} (Time: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">log</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">timestamp</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">})\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  }</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">});</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Output: Error Detection: Connection failed (Time: 3)</span></span></code></pre></div></li></ol><h2 id="🧠-practical-code-example-product-search" tabindex="-1">🧠 Practical Code Example (Product Search) <a class="header-anchor" href="#🧠-practical-code-example-product-search" aria-label="Permalink to &quot;🧠 Practical Code Example (Product Search)&quot;">​</a></h2><p>This is an example of searching for products that meet specific criteria from the inventory.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts</span></span>
<span class="line"><span>import { from, fromEvent } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { find } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>interface Product {</span></span>
<span class="line"><span>  id: string;</span></span>
<span class="line"><span>  name: string;</span></span>
<span class="line"><span>  price: number;</span></span>
<span class="line"><span>  category: string; }</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const products: Product[] = [</span></span>
<span class="line"><span>  { id: &#39;P1&#39;, name: &#39;Wireless Mouse&#39;, price: 2980, category: &#39;PC Peripherals&#39; }</span></span>
<span class="line"><span>  { id: &#39;P2&#39;, name: &#39;Mechanical Keyboard&#39;, price: 8980, category: &#39;PC Peripherals&#39; }, }</span></span>
<span class="line"><span>  { id: &#39;P3&#39;, name: &#39;USB flash drive 64GB&#39;, price: 1480, category: &#39;Storage&#39; }</span></span>
<span class="line"><span>  { id: &#39;P4&#39;, name: &#39;monitor 27inch&#39;, price: 29800, category: &#39;Displays&#39; }, { id: &#39;P4&#39;, name: &#39;monitor 27inch&#39;, price: 29800, category: &#39;Displays&#39; }</span></span>
<span class="line"><span>  { id: &#39;P5&#39;, name: &#39;laptop stand&#39;, price: 3980, category: &#39;PC peripherals&#39; }</span></span>
<span class="line"><span>];</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Create UI elements</span></span>
<span class="line"><span>const container = document.createElement(&#39;div&#39;);</span></span>
<span class="line"><span>document.body.appendChild(container);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const title = document.createElement(&#39;h3&#39;);</span></span>
<span class="line"><span>title.textContent = &#39;Product Search&#39;;</span></span>
<span class="line"><span>container.appendChild(title);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const input = document.createElement(&#39;input&#39;);</span></span>
<span class="line"><span>input.type = &#39;number&#39;;</span></span>
<span class="line"><span>input.placeholder = &#39;Enter maximum price&#39;;</span></span>
<span class="line"><span>input.style.marginRight = &#39;10px&#39;;</span></span>
<span class="line"><span>container.appendChild(input);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const searchButton = document.createElement(&#39;button&#39;);</span></span>
<span class="line"><span>searchButton.textContent = &#39;search&#39;;</span></span>
<span class="line"><span>container.appendChild(searchButton);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const result = document.createElement(&#39;div&#39;);</span></span>
<span class="line"><span>result.style.marginTop = &#39;10px&#39;;</span></span>
<span class="line"><span>container.appendChild(result);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// search processing</span></span>
<span class="line"><span>// Note: originally the recommended pattern is to flatten with a switchMap,</span><span> // but here we use UI validation (early return),</span></span>
<span class="line"><span>Note: Although the recommended pattern is to flatten with a switchMap, // here we nest subscribe for readability, since it includes UI validation (early return).</span></span>
<span class="line"><span>// Consider a flat implementation using \`switchMap\` in production code.</span></span>
<span class="line"><span>fromEvent(searchButton, &#39;click&#39;).subscribe(() =&gt; {</span></span>
<span class="line"><span>  const maxPrice = parseInt(input.value);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>  if (isNaN(maxPrice)) {</span></span>
<span class="line"><span>    result.textContent = &#39;Please enter a price&#39;;</span></span>
<span class="line"><span>    result.style.color = &#39;red&#39;;</span></span>
<span class="line"><span>    return;</span></span>
<span class="line"><span>  }</span></span>
<span class="line"><span></span></span>
<span class="line"><span>  // nest subscribe: originally recommended to flatten with switchMap</span></span>
<span class="line"><span>  from(products).pipe(</span></span>
<span class="line"><span>    find(product =&gt; product.price &lt;= maxPrice)</span></span>
<span class="line"><span>  ).subscribe(product =&gt; {</span></span>
<span class="line"><span>    if (product) {</span></span>
<span class="line"><span>      result.innerHTML = \`</span></span>
<span class="line"><span>        &lt;strong&gt;Found! &lt;/strong&gt;&lt;br</span></span>
<span class="line"><span>        product name: \${product.name}&lt;br&gt;</span></span>
<span class="line"><span>        Price: \${product.price.toLocaleString()}&lt;br&gt;</span></span>
<span class="line"><span>        Category: \${product.category}</span></span>
<span class="line"><span>      \`;</span></span>
<span class="line"><span>      result.style.color = &#39;green&#39;;</span></span>
<span class="line"><span>    } else {</span></span>
<span class="line"><span>      result.textContent = \`¥\${maxPrice.toLocaleString()} or less product not found \`;</span></span>
<span class="line"><span>      result.style.color = &#39;orange&#39;; }</span></span>
<span class="line"><span>    }</span></span>
<span class="line"><span>  });</span></span>
<span class="line"><span>});</span></span></code></pre></div><p>This code searches for and displays the first item below the price entered by the user.</p><h2 id="🎯-filter-the-difference-between" tabindex="-1">🎯 filter The difference between <a class="header-anchor" href="#🎯-filter-the-difference-between" aria-label="Permalink to &quot;🎯 filter The difference between&quot;">​</a></h2><p><code>find</code> and <code>filter</code> are used for different purposes.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts</span></span>
<span class="line"><span>import { from } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { find, filter } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// filter: output all matching values</span></span>
<span class="line"><span>numbers$.pipe(</span></span>
<span class="line"><span>  filter(n =&gt; n &gt; 5)</span></span>
<span class="line"><span>).subscribe({</span></span>
<span class="line"><span>  next: console.log,</span></span>
<span class="line"><span>  complete: () =&gt; console.log(&#39;filter complete&#39;)</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// output: 7, 8, 9, 10, filter complete</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// find: output only the first value that matches the condition</span></span>
<span class="line"><span>numbers$.pipe(</span></span>
<span class="line"><span>  find(n =&gt; n &gt; 5)</span></span>
<span class="line"><span>).subscribe({</span></span>
<span class="line"><span>  next: console.log,</span></span>
<span class="line"><span>  complete: () =&gt; console.log(&#39;find completed&#39;)</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// output: 7, find complete</span></span></code></pre></div><table tabindex="0"><thead><tr><th>Operator</th><th>Number of outputs</th><th>Completion timing</th><th>Use case</th></tr></thead><tbody><tr><td><code>filter(predicate)</code></td><td>All values matching the condition</td><td>At completion of original stream</td><td>Data Refinement</td></tr><tr><td><code>find(predicate)</code></td><td>Only the first value matching the criteria</td><td>Immediately upon discovery</td><td>Search and confirm existence</td></tr></tbody></table><h2 id="📋-type-safe-usage" tabindex="-1">📋 Type-safe usage <a class="header-anchor" href="#📋-type-safe-usage" aria-label="Permalink to &quot;📋 Type-safe usage&quot;">​</a></h2><p>TypeScript This is an example of a type-safe implementation that utilizes generics in</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>\`\`\`ts</span></span>
<span class="line"><span>import { Observable, from } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { find } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>interface Task {</span></span>
<span class="line"><span>  id: number;</span></span>
<span class="line"><span>  title: string;</span></span>
<span class="line"><span>  completed: boolean;</span></span>
<span class="line"><span>  priority: &#39;high&#39; | &#39;medium&#39; | &#39;low&#39;; }</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>function findTaskById(</span></span>
<span class="line"><span>  tasks$: Observable&lt;Task&gt;,</span></span>
<span class="line"><span>  id: number</span></span>
<span class="line"><span>): Observable&lt;Task | undefined&gt; {</span></span>
<span class="line"><span>  return tasks$.pipe(</span></span>
<span class="line"><span>    find(task =&gt; task.id === id)</span></span>
<span class="line"><span>  ); }</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>function findFirstIncompleteTask(</span></span>
<span class="line"><span>  tasks$: Observable&lt;Task&gt;</span></span>
<span class="line"><span>): Observable&lt;Task | undefined&gt; {</span></span>
<span class="line"><span>  return tasks$.pipe(</span></span>
<span class="line"><span>    find(task =&gt; !task.completed)</span></span>
<span class="line"><span>  );</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Example usage</span></span>
<span class="line"><span>const tasks$ = from([</span></span>
<span class="line"><span>  { id: 1, title: &#39;Task A&#39;, completed: true, priority: &#39;high&#39; as const }</span></span>
<span class="line"><span>  { id: 2, title: &#39;Task B&#39;, completed: false, priority: &#39;medium&#39; as const }</span></span>
<span class="line"><span>  { id: 3, title: &#39;Task C&#39;, completed: false, priority: &#39;low&#39; as const }</span></span>
<span class="line"><span>] as Task[]);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Search by ID</span></span>
<span class="line"><span>findTaskById(tasks$, 2).subscribe(task =&gt; {</span></span>
<span class="line"><span>  if (task) {</span></span>
<span class="line"><span>    console.log(\`found: \${task.title}\`);</span></span>
<span class="line"><span>  } else {</span></span>
<span class="line"><span>    console.log(&#39;Task not found&#39;); }</span></span>
<span class="line"><span>  }</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// Output: Found: Task B</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Find uncompleted tasks</span></span>
<span class="line"><span>findFirstIncompleteTask(tasks$).subscribe(task =&gt; {</span></span>
<span class="line"><span>  if (task) {</span></span>
<span class="line"><span>    console.log(\`Next task: \${task.title} (priority: \${task.priority})\`);</span></span>
<span class="line"><span>  }</span></span>
<span class="line"><span>}); })</span></span>
<span class="line"><span>// Output: Next task: Task B (priority: medium)</span></span></code></pre></div><h2 id="🔄-find-and-findindex-the-difference-between" tabindex="-1">🔄 find and findIndex The difference between <a class="header-anchor" href="#🔄-find-and-findindex-the-difference-between" aria-label="Permalink to &quot;🔄 find and findIndex The difference between&quot;">​</a></h2><p>RxJSin the <code>findIndex</code> operator is also available.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts</span></span>
<span class="line"><span>import { from } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { find, findIndex } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const numbers$ = from([10, 20, 30, 40, 50]);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// find: return a value</span></span>
<span class="line"><span>numbers$.pipe(</span></span>
<span class="line"><span>  find(n =&gt; n &gt; 25)</span></span>
<span class="line"><span>).subscribe(console.log);</span></span>
<span class="line"><span>// output: 30</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// findIndex: return index</span></span>
<span class="line"><span>numbers$.pipe(</span></span>
<span class="line"><span>  findIndex(n =&gt; n &gt; 25)</span></span>
<span class="line"><span>).subscribe(console.log);</span></span>
<span class="line"><span>// output: 2 (index of 30)</span></span></code></pre></div><table tabindex="0"><thead><tr><th>Operator</th><th>Return value</th><th>If not found</th></tr></thead><tbody><tr><td><code>find(predicate)</code></td><td>Value itself</td><td><code>undefined</code></td></tr><tr><td><code>findIndex(predicate)</code></td><td>Index (numeric)</td><td><code>-1</code></td></tr></tbody></table><h2 id="⚠️-common-mistakes" tabindex="-1">⚠️ Common mistakes <a class="header-anchor" href="#⚠️-common-mistakes" aria-label="Permalink to &quot;⚠️ Common mistakes&quot;">​</a></h2><div class="note custom-block github-alert"><p class="custom-block-title">NOTE</p><p><code>find</code> outputs <code>undefined</code> is output when a value is not found. This is not an error. If you need an error, use <code>first</code> to be used.</p></div><h3 id="error-expect-error-handling-if-value-not-found" tabindex="-1">Error: Expect error handling if value not found <a class="header-anchor" href="#error-expect-error-handling-if-value-not-found" aria-label="Permalink to &quot;Error: Expect error handling if value not found&quot;">​</a></h3><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts</span></span>
<span class="line"><span>import { from } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { find } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const numbers$ = from([1, 3, 5, 7]);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ❌ Bad example: expecting error handling but not called</span></span>
<span class="line"><span>numbers$.pipe(</span></span>
<span class="line"><span>  find(n =&gt; n &gt; 10)</span></span>
<span class="line"><span>).subscribe({</span></span>
<span class="line"><span>  next: console.log,.</span></span>
<span class="line"><span>  error: err =&gt; console.log(&#39;error:&#39;, err) // not called</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// output: undefined</span></span></code></pre></div><h3 id="positive-undefined-check-for-or-first-use" tabindex="-1">Positive: undefined Check for or first Use <a class="header-anchor" href="#positive-undefined-check-for-or-first-use" aria-label="Permalink to &quot;Positive: undefined Check for or first Use&quot;">​</a></h3><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts</span></span>
<span class="line"><span>import { from } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { find, first } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const numbers$ = from([1, 3, 5, 7]);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ✅ Good example 1: check for undefined</span></span>
<span class="line"><span>numbers$.pipe(</span></span>
<span class="line"><span>  find(n =&gt; n &gt; 10)</span></span>
<span class="line"><span>).subscribe(result =&gt; {</span></span>
<span class="line"><span>  if (result ! == undefined) {</span></span>
<span class="line"><span>    console.log(&#39;found:&#39;, result); }</span></span>
<span class="line"><span>  } else {</span></span>
<span class="line"><span>    console.log(&#39;Not found:&#39;); }</span></span>
<span class="line"><span>  }</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// Output: not found</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ✅ Good example 2: use first if you need an error</span></span>
<span class="line"><span>numbers$.pipe(</span></span>
<span class="line"><span>  first(n =&gt; n &gt; 10, 0) // specify default value</span></span>
<span class="line"><span>).subscribe({</span></span>
<span class="line"><span>  next: console.log, error: err =&gt; console.log</span></span>
<span class="line"><span>  error: err =&gt; console.log(&#39;Error:&#39;, err.message)</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// output: 0</span></span></code></pre></div><h2 id="🎓-summary" tabindex="-1">🎓 Summary <a class="header-anchor" href="#🎓-summary" aria-label="Permalink to &quot;🎓 Summary&quot;">​</a></h2><h3 id="when-you-should-use-find" tabindex="-1">When you should use find <a class="header-anchor" href="#when-you-should-use-find" aria-label="Permalink to &quot;When you should use find&quot;">​</a></h3><ul><li>✅ You want to find the first value that satisfies a condition</li><li>✅ When you want to check for the existence of a value</li><li>✅ When you want to treat a value as <code>undefined</code> if it is not found.</li><li>✅ When you want to find a specific element in an array or list</li></ul><h3 id="when-you-should-use-first" tabindex="-1">When you should use first <a class="header-anchor" href="#when-you-should-use-first" aria-label="Permalink to &quot;When you should use first&quot;">​</a></h3><ul><li>✅ If you want to get the first value</li><li>✅ If you want to issue an error when a value is not found</li></ul><h3 id="filter-should-be-used-if" tabindex="-1">filter should be used if <a class="header-anchor" href="#filter-should-be-used-if" aria-label="Permalink to &quot;filter should be used if&quot;">​</a></h3><ul><li>✅ If you need all values that match the criteria</li><li>✅ If you want to narrow down the data</li></ul><h3 id="notes" tabindex="-1">Notes <a class="header-anchor" href="#notes" aria-label="Permalink to &quot;Notes&quot;">​</a></h3><ul><li>⚠️ <code>find</code> outputs <code>undefined</code> if not found (not an error)</li><li>⚠️ Complete immediately with the first value that satisfies the condition</li><li>⚠️ TypeScript will return a return value of type <code>T | undefined</code>.</li></ul><h2 id="🚀-next-steps" tabindex="-1">🚀 Next Steps <a class="header-anchor" href="#🚀-next-steps" aria-label="Permalink to &quot;🚀 Next Steps&quot;">​</a></h2><ul><li><strong>[first](. /first)</strong> - learn how to get the first value</li><li><strong>[filter](. /filter)</strong> - learn how to filter based on conditions</li><li><strong><a href="https://rxjs.dev/api/operators/findIndex" target="_blank" rel="noreferrer">findIndex</a></strong> - learn how to get the index of the first value that satisfies a condition (official documentation)</li><li><strong>[filtering-operator-practical-use-cases](. /practical-use-cases)</strong> - learn real use cases</li></ul>`,47)])])}const E=a(t,[["render",e]]);export{c as __pageData,E as default};
