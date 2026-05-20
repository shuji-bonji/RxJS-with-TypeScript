import{_ as n,o as i,c as a,a2 as e}from"./chunks/framework.B0tZAgFO.js";const E=JSON.parse('{"title":"find - findet den ersten Wert, der die Bedingung erfüllt","description":"find ist ein RxJS-Filteroperator, der den ersten Wert findet, der eine Bedingung erfüllt, und diesen ausgibt, wodurch der Stream sofort vervollständigt wird. Er eignet sich ideal für Situationen, in denen Sie ein bestimmtes Element aus einem Array oder einer Liste finden möchten, z. B. bei der Suche nach Benutzern, der Überprüfung des Inventars oder der Erkennung von Fehlerprotokollen. Wenn kein Wert gefunden wird, wird ein undefinierter Wert ausgegeben, und in TypeScript ist der Rückgabewert vom Typ T | undefiniert.","frontmatter":{"description":"find ist ein RxJS-Filteroperator, der den ersten Wert findet, der eine Bedingung erfüllt, und diesen ausgibt, wodurch der Stream sofort vervollständigt wird. Er eignet sich ideal für Situationen, in denen Sie ein bestimmtes Element aus einem Array oder einer Liste finden möchten, z. B. bei der Suche nach Benutzern, der Überprüfung des Inventars oder der Erkennung von Fehlerprotokollen. Wenn kein Wert gefunden wird, wird ein undefinierter Wert ausgegeben, und in TypeScript ist der Rückgabewert vom Typ T | undefiniert."},"headers":[],"relativePath":"de/guide/operators/filtering/find.md","filePath":"de/guide/operators/filtering/find.md","lastUpdated":1779269732000}'),p={name:"de/guide/operators/filtering/find.md"};function l(t,s,h,r,k,d){return i(),a("div",null,[...s[0]||(s[0]=[e(`<h1 id="find-findet-den-ersten-wert-der-die-bedingung-erfullt" tabindex="-1">find - findet den ersten Wert, der die Bedingung erfüllt <a class="header-anchor" href="#find-findet-den-ersten-wert-der-die-bedingung-erfullt" aria-label="Permalink to &quot;find - findet den ersten Wert, der die Bedingung erfüllt&quot;">​</a></h1><p>Der Operator &quot;find&quot; findet und gibt den <strong>ersten Wert aus, der die Bedingung erfüllt</strong> und schließt den Stream sofort ab. Wenn kein Wert gefunden wird, gibt er <code>undefined</code> aus.</p><h2 id="🔰-grundlegende-syntax-und-verwendung" tabindex="-1">🔰 Grundlegende Syntax und Verwendung <a class="header-anchor" href="#🔰-grundlegende-syntax-und-verwendung" aria-label="Permalink to &quot;🔰 Grundlegende Syntax und Verwendung&quot;">​</a></h2><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { from } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { find } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> numbers$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> from</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">([</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">1</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">3</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">5</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">7</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">8</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">9</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">10</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">]);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">numbers$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  find</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">n</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> n </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">%</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 2</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> ===</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 0</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Ausgabe.: 8(erste gerade Zahl)</span></span></code></pre></div><p><strong>Ablauf der Operation</strong>:.</p><ol><li>check 1, 3, 5, 7 → Bedingung nicht erfüllt</li><li>check 8 → Bedingung erfüllt → Ausgabe 8 und vollständig</li><li>9, 10 nicht ausgewertet</li></ol><p><a href="https://rxjs.dev/api/operators/find" target="_blank" rel="noreferrer">🌐 Offizielle RxJS Dokumentation - <code>find</code></a></p><h2 id="🆚-kontrast-zu-first" tabindex="-1">🆚 Kontrast zu first <a class="header-anchor" href="#🆚-kontrast-zu-first" aria-label="Permalink to &quot;🆚 Kontrast zu first&quot;">​</a></h2><p><code>find</code> und <code>first</code> sind ähnlich, aber ihre Verwendung ist unterschiedlich.</p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { from } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { find, first } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> numbers$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> from</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">([</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">1</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">3</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">5</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">7</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">8</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">9</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">10</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">]);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// first: Erster Wert, der die Bedingung erfüllt (Bedingung ist optional)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">numbers$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  first</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">n</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> n </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">&gt;</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 5</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Ausgabe.: 7</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// find: Erster Wert, der die Bedingung erfüllt (Bedingung ist obligatorisch)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">numbers$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  find</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">n</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> n </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">&gt;</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 5</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Ausgabe.: 7</span></span></code></pre></div><table tabindex="0"><thead><tr><th>Bediener.</th><th>Angabe der Bedingung</th><th>Wenn kein Wert gefunden wird</th><th>Anwendungsfall.</th></tr></thead><tbody><tr><td><code>first()</code></td><td>Option</td><td>Fehler (<code>EmptyError</code>)</td><td>Ermittelt den ersten Wert</td></tr><tr><td>first(Prädikat)\`</td><td>Optional</td><td>Fehler (<code>EmptyError</code>)</td><td>Bedingtes Erhalten.</td></tr><tr><td>find(Prädikat)\`</td><td>Erforderlich.</td><td>Ausgabe <code>Undefiniert</code>.</td><td>Suche und Existenzprüfung</td></tr></tbody></table><h2 id="💡-typisches-nutzungsmuster" tabindex="-1">💡 Typisches Nutzungsmuster <a class="header-anchor" href="#💡-typisches-nutzungsmuster" aria-label="Permalink to &quot;💡 Typisches Nutzungsmuster&quot;">​</a></h2><ol><li><strong>Benutzersuche</strong>.</li></ol><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">   import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { from } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
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
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">   // ID(Bedingung ist optional)2Suche nach Benutzern mit</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   users$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">     find</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">user</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> user.id </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">===</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 2</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   ).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">user</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">     if</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> (user) {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">       console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`Gefunden: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">user</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">name</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">     } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">else</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">       console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Benutzer nicht gefunden&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">     }</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   });</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">   // Ausgabe.: Gefunden: Bob</span></span></code></pre></div><ol start="2"><li><p><strong>Inventar prüfen</strong></p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { from } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
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
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  { id: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;A2&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, name: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Maus&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, stock: </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">15</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> },</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  { id: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;A3&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, name: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Tastaturen&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, stock: </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">8</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> }</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">] </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">as</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> Product</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">[]);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Finde heraus, was nicht vorrätig ist</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">products$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  find</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">product</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> product.stock </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">===</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 0</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">product</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">  if</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> (product) {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">    console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`Nicht auf Lager: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">product</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">name</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">else</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">    console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Alle auf Lager&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  }</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">});</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Ausgabe.: Nicht auf Lager: NotebookPC</span></span></code></pre></div></li><li><p><strong>Suche im Fehlerprotokoll</strong></p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { from } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
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
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Suche nach dem ersten Fehler</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">logs$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  find</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">log</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> log.level </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">===</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;error&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">log</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">  if</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> (log) {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">    console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`Fehlererkennung: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">log</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">message</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">} (Zeit: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">log</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">timestamp</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">})\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  }</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">});</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Ausgabe.: Fehlererkennung: Connection failed (Zeit: 3)</span></span></code></pre></div></li></ol><h2 id="🧠-praktisches-code-beispiel-produktsuche" tabindex="-1">🧠 Praktisches Code-Beispiel (Produktsuche) <a class="header-anchor" href="#🧠-praktisches-code-beispiel-produktsuche" aria-label="Permalink to &quot;🧠 Praktisches Code-Beispiel (Produktsuche)&quot;">​</a></h2><p>Dies ist ein Beispiel für die Suche nach Produkten aus dem Bestand, die bestimmten Kriterien entsprechen.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { from, fromEvent } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>importiere { find } von &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>Schnittstelle Product {</span></span>
<span class="line"><span>  id: string;</span></span>
<span class="line"><span>  name: string;</span></span>
<span class="line"><span>  Preis: Zahl;</span></span>
<span class="line"><span>  Kategorie: string;</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const products: Product[] = [</span></span>
<span class="line"><span>  { id: &#39;P1&#39;, Name: &#39;Kabellose Maus&#39;, Preis: 2980, Kategorie: &#39;PC-Peripheriegeräte&#39; }</span></span>
<span class="line"><span>  { id: &#39;P2&#39;, name: &#39;Mechanische Tastatur&#39;, preis: 8980, kategorie: &#39;PC-Peripheriegeräte&#39; }</span></span>
<span class="line"><span>  { id: &#39;P3&#39;, name: &#39;USB-Speicherstick 64GB&#39;, preis: 1480, kategorie: &#39;Speicher&#39; }</span></span>
<span class="line"><span>  { id: &#39;P4&#39;, name: &#39;Monitor 27-Zoll&#39;, preis: 29800, kategorie: &#39;Bildschirme&#39; }</span></span>
<span class="line"><span>  { id: &#39;P5&#39;, name: &#39;Laptop-Ständer&#39;, preis: 3980, kategorie: &#39;PC-Peripheriegeräte&#39; }</span></span>
<span class="line"><span>];</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Erstellen von UI-Elementen</span></span>
<span class="line"><span>const container = document.createElement(&#39;div&#39;);.</span></span>
<span class="line"><span>document.body.appendChild(container);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const title = document.createElement(&#39;h3&#39;);</span></span>
<span class="line"><span>title.textContent = &#39;Produktsuche&#39;;</span></span>
<span class="line"><span>container.appendChild(title);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const input = document.createElement(&#39;input&#39;);</span></span>
<span class="line"><span>input.type = &#39;Zahl&#39;;</span></span>
<span class="line"><span>input.placeholder = &#39;Höchstpreis eingeben&#39;;</span></span>
<span class="line"><span>input.style.marginRight = &#39;10px&#39;;</span></span>
<span class="line"><span>container.appendChild(input);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const searchButton = document.createElement(&#39;button&#39;);</span></span>
<span class="line"><span>searchButton.textContent = &#39;suche&#39;;</span></span>
<span class="line"><span>container.appendChild(searchButton);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const result = document.createElement(&#39;div&#39;);</span></span>
<span class="line"><span>result.style.marginTop = &#39;10px&#39;;</span></span>
<span class="line"><span>container.appendChild(result);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Suchverarbeitung</span></span>
<span class="line"><span>// Hinweis: Ursprünglich ist das empfohlene Muster, mit einer switchMap zu flatten, aber,</span></span>
<span class="line"><span>// Hinweis: Obwohl das empfohlene Muster die Verflachung mit einer switchMap ist,</span><span> // verschachteln wir hier das subscribe aus Gründen der Lesbarkeit,</span><span> // weil es eine UI-Validierung (frühe Rückkehr) enthält.</span></span>
<span class="line"><span>// Betrachten Sie eine flache Implementierung mit \`switchMap\` im Produktionscode.</span></span>
<span class="line"><span>fromEvent(searchButton, &#39;click&#39;).subscribe(() =&gt; {</span></span>
<span class="line"><span>  const maxPrice = parseInt(input.value);.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>  if (isNaN(maxPreis)) {</span></span>
<span class="line"><span>    result.textContent = &#39;Bitte geben Sie einen Preis ein&#39;;</span></span>
<span class="line"><span>    result.style.color = &#39;rot&#39;;</span></span>
<span class="line"><span>    zurück;</span></span>
<span class="line"><span>  }</span></span>
<span class="line"><span></span></span>
<span class="line"><span>  // Nest subscribe: ursprünglich empfohlen, mit switchMap zu flatten</span></span>
<span class="line"><span>  from(Produkte).pipe(</span></span>
<span class="line"><span>    find(produkt =&gt; produkt.preis &lt;= maxPreis)</span></span>
<span class="line"><span>  ).subscribe(produkt =&gt; {</span></span>
<span class="line"><span>    if (produkt) {</span></span>
<span class="line"><span>      result.innerHTML = \`</span></span>
<span class="line"><span>        &lt;strong&gt;Gefunden! &lt;/strong&gt;&lt;br&gt;</span></span>
<span class="line"><span>        Produktname: \${Produkt.Name}&lt;br&gt;</span></span>
<span class="line"><span>        Preis: \${Produkt.Preis.toLocaleString()}&lt;br&gt;</span></span>
<span class="line"><span>        Kategorie: \${product.category}</span></span>
<span class="line"><span>      \`;</span></span>
<span class="line"><span>      result.style.color = &#39;grün&#39;;</span></span>
<span class="line"><span>    } else {</span></span>
<span class="line"><span>      result.textContent = \`¥\${maxPrice.toLocaleString()} oder weniger Produkt nicht gefunden \`;</span></span>
<span class="line"><span>      result.style.color = &#39;orange&#39;; }</span></span>
<span class="line"><span>    }</span></span>
<span class="line"><span>  });</span></span>
<span class="line"><span>});</span></span></code></pre></div><p>Dieser Code sucht nach dem ersten Produkt unter dem vom Benutzer eingegebenen Preis und zeigt es an.</p><h2 id="🎯-filter-der-unterschied-zwischen" tabindex="-1">🎯 filter Der Unterschied zwischen <a class="header-anchor" href="#🎯-filter-der-unterschied-zwischen" aria-label="Permalink to &quot;🎯 filter Der Unterschied zwischen&quot;">​</a></h2><p><code>find</code> und <code>filter</code> werden für unterschiedliche Zwecke verwendet.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { from } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>importiere { find, filter } von &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// filter: Ausgabe aller Werte, die die Bedingung erfüllen</span></span>
<span class="line"><span>numbers$.pipe(</span></span>
<span class="line"><span>  filter(n =&gt; n &gt; 5)</span></span>
<span class="line"><span>).subscribe({</span></span>
<span class="line"><span>  next: console.log,.</span></span>
<span class="line"><span>  complete: () =&gt; console.log(&#39;filter complete&#39;)</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// Ausgabe: 7, 8, 9, 10, filter complete</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// find: Ausgabe nur des ersten Wertes, der die Bedingung erfüllt</span></span>
<span class="line"><span>numbers$.pipe(</span></span>
<span class="line"><span>  find(n =&gt; n &gt; 5)</span></span>
<span class="line"><span>).subscribe({</span></span>
<span class="line"><span>  next: console.log,.</span></span>
<span class="line"><span>  complete: () =&gt; console.log(&#39;find complete&#39;)</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// Ausgabe: 7, find complete</span></span></code></pre></div><table tabindex="0"><thead><tr><th>Operator</th><th>Anzahl der Ausgaben</th><th>Zeitpunkt der Fertigstellung</th><th>Anwendungsfall</th></tr></thead><tbody><tr><td><code>filter(predicate)</code></td><td>Alle Werte, die die Bedingung erfüllen</td><td>Bei Abschluss des ursprünglichen Datenstroms</td><td>Verfeinerung der Daten</td></tr><tr><td><code>find(predicate)</code></td><td>Nur der erste Wert, der die Kriterien erfüllt</td><td>Unmittelbar nach der Entdeckung</td><td>Suche und Existenzprüfung</td></tr></tbody></table><h2 id="📋-typsichere-verwendung" tabindex="-1">📋 Typsichere Verwendung <a class="header-anchor" href="#📋-typsichere-verwendung" aria-label="Permalink to &quot;📋 Typsichere Verwendung&quot;">​</a></h2><p>TypeScript Dies ist ein Beispiel für eine typsichere Implementierung, die die Generika in</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { Observable, from } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>importiere { find } von &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>Schnittstelle Task {</span></span>
<span class="line"><span>  id: Zahl;</span></span>
<span class="line"><span>  title: string;</span></span>
<span class="line"><span>  completed: boolescher Wert;</span></span>
<span class="line"><span>  Priorität: &#39;hoch&#39; | &#39;mittel&#39; | &#39;niedrig&#39;; }</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>function findTaskById(</span></span>
<span class="line"><span>  tasks$: Observable,.</span></span>
<span class="line"><span>  id: Zahl</span></span>
<span class="line"><span>): Observable | undefined&gt; {</span></span>
<span class="line"><span>  return tasks$.pipe(</span></span>
<span class="line"><span>    find(aufgabe =&gt; aufgabe.id === id)</span></span>
<span class="line"><span>  );</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>function findFirstIncompleteTask(</span></span>
<span class="line"><span>  tasks$: Observable</span></span>
<span class="line"><span>): Observable | undefined&gt; {</span></span>
<span class="line"><span>  return tasks$.pipe(</span></span>
<span class="line"><span>    find(task =&gt; !task.completed)</span></span>
<span class="line"><span>  );</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Beispiel für die Verwendung</span></span>
<span class="line"><span>const tasks$ = from([.</span></span>
<span class="line"><span>  { id: 1, title: &#39;Aufgabe A&#39;, completed: true, priority: &#39;high&#39; as const }</span></span>
<span class="line"><span>  { id: 2, title: &#39;Aufgabe B&#39;, completed: false, priority: &#39;medium&#39; as const }</span></span>
<span class="line"><span>  { id: 3, title: &#39;Aufgabe C&#39;, completed: false, priority: &#39;low&#39; as const }</span></span>
<span class="line"><span>] as Task[]);.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Suche nach ID</span></span>
<span class="line"><span>findTaskById(tasks$, 2).subscribe(task =&gt; {</span></span>
<span class="line"><span>  if (Aufgabe) {</span></span>
<span class="line"><span>    console.log(\`Gefunden: \${task.title}\`);</span></span>
<span class="line"><span>  } else {</span></span>
<span class="line"><span>    console.log(&#39;Aufgabe nicht gefunden&#39;); }</span></span>
<span class="line"><span>  }</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// Ausgabe: gefunden: Aufgabe B</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Unerledigte Aufgaben finden</span></span>
<span class="line"><span>findFirstIncompleteTask(tasks$).subscribe(task =&gt; {</span></span>
<span class="line"><span>  if (Aufgabe) {</span></span>
<span class="line"><span>    console.log(\`Nächste Aufgabe: \${task.title} (Priorität: \${task.priority})\`);</span></span>
<span class="line"><span>  }</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// Ausgabe: nächste Aufgabe: Aufgabe B (Priorität: mittel)</span></span></code></pre></div><h2 id="🔄-find-und-findindex-der-unterschied-zwischen" tabindex="-1">🔄 find und findIndex Der Unterschied zwischen <a class="header-anchor" href="#🔄-find-und-findindex-der-unterschied-zwischen" aria-label="Permalink to &quot;🔄 find und findIndex Der Unterschied zwischen&quot;">​</a></h2><p>RxJSin den <code>findIndex</code> Operatoren sind ebenfalls verfügbar.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts</span></span>
<span class="line"><span>import { from } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>importiere { find, findIndex } von &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const numbers$ = from([10, 20, 30, 40, 50]);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// find: Rückgabe eines Wertes</span></span>
<span class="line"><span>numbers$.pipe(</span></span>
<span class="line"><span>  find(n =&gt; n &gt; 25)</span></span>
<span class="line"><span>).subscribe(console.log);.</span></span>
<span class="line"><span>// Ausgabe: 30</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// findIndex: Rückgabe index</span></span>
<span class="line"><span>numbers$.pipe(</span></span>
<span class="line"><span>  findIndex(n =&gt; n &gt; 25)</span></span>
<span class="line"><span>).subscribe(console.log);.</span></span>
<span class="line"><span>// Ausgabe: 2 (Index von 30)</span></span></code></pre></div><table tabindex="0"><thead><tr><th>Operator</th><th>Rückgabewert</th><th>wenn der Wert nicht gefunden wird</th></tr></thead><tbody><tr><td><code>find(predicate)</code></td><td>Wert selbst</td><td><code>undefined</code></td></tr><tr><td><code>findIndex(predicate)</code></td><td>Index (numerischer Wert)</td><td><code>-1</code></td></tr></tbody></table><h2 id="⚠️-haufige-fehler" tabindex="-1">⚠️ Häufige Fehler <a class="header-anchor" href="#⚠️-haufige-fehler" aria-label="Permalink to &quot;⚠️ Häufige Fehler&quot;">​</a></h2><div class="note custom-block github-alert"><p class="custom-block-title">NOTE</p><p><code>find</code> wenn der Wert nicht gefunden wird. <code>undefined</code> ausgegeben wird. Dies führt nicht zu einem Fehler. Wenn ein Fehler erforderlich ist, muss <code>first</code> verwendet werden.</p></div><h3 id="fehler-erwartete-fehlerbehandlung-wenn-der-wert-nicht-gefunden-wird" tabindex="-1">Fehler.: Erwartete Fehlerbehandlung, wenn der Wert nicht gefunden wird. <a class="header-anchor" href="#fehler-erwartete-fehlerbehandlung-wenn-der-wert-nicht-gefunden-wird" aria-label="Permalink to &quot;Fehler.: Erwartete Fehlerbehandlung, wenn der Wert nicht gefunden wird.&quot;">​</a></h3><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { from } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>importiere { find } von &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const numbers$ = from([1, 3, 5, 7]);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ❌ Schlechtes Beispiel: Fehlerbehandlung erwartet, aber nicht aufgerufen</span></span>
<span class="line"><span>numbers$.pipe(</span></span>
<span class="line"><span>  find(n =&gt; n &gt; 10)</span></span>
<span class="line"><span>).subscribe({</span></span>
<span class="line"><span>  next: console.log,.</span></span>
<span class="line"><span>  error: err =&gt; console.log(&#39;Error:&#39;, err) // nicht aufgerufen</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// Ausgabe: undefiniert</span></span></code></pre></div><h3 id="positiv-undefined-prufen-sie-auf-oder-first-verwendung-des" tabindex="-1">Positiv: undefined Prüfen Sie auf oder first Verwendung des <a class="header-anchor" href="#positiv-undefined-prufen-sie-auf-oder-first-verwendung-des" aria-label="Permalink to &quot;Positiv: undefined Prüfen Sie auf oder first Verwendung des&quot;">​</a></h3><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { from } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>importiere { find, first } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const numbers$ = from([1, 3, 5, 7]);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ✅ Gutes Beispiel 1: Prüfung auf undefiniert</span></span>
<span class="line"><span>numbers$.pipe(</span></span>
<span class="line"><span>  find(n =&gt; n &gt; 10)</span></span>
<span class="line"><span>).subscribe(result =&gt; {</span></span>
<span class="line"><span>  if (result ! == undefiniert) {</span></span>
<span class="line"><span>    console.log(&#39;Gefunden:&#39;, Ergebnis);</span></span>
<span class="line"><span>  } else {</span></span>
<span class="line"><span>    console.log(&#39;Nicht gefunden:&#39;); }</span></span>
<span class="line"><span>  }</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// Ausgabe: nicht gefunden</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ✅ Gutes Beispiel 2: Verwenden Sie das erste, wenn Sie einen Fehler benötigen</span></span>
<span class="line"><span>numbers$.pipe(</span></span>
<span class="line"><span>  first(n =&gt; n &gt; 10, 0) // Standardwert angeben</span></span>
<span class="line"><span>).subscribe({</span></span>
<span class="line"><span>  next: console.log,.</span></span>
<span class="line"><span>  error: err =&gt; console.log(&#39;Fehler:&#39;, err.message)</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// Ausgabe: 0</span></span></code></pre></div><h2 id="🎓-zusammenfassung" tabindex="-1">🎓 Zusammenfassung <a class="header-anchor" href="#🎓-zusammenfassung" aria-label="Permalink to &quot;🎓 Zusammenfassung&quot;">​</a></h2><h3 id="wann-sie-find-verwenden-sollten" tabindex="-1">Wann Sie find verwenden sollten. <a class="header-anchor" href="#wann-sie-find-verwenden-sollten" aria-label="Permalink to &quot;Wann Sie find verwenden sollten.&quot;">​</a></h3><ul><li>✅ Wenn Sie den ersten Wert finden wollen, der eine Bedingung erfüllt</li><li>✅ Wenn Sie das Vorhandensein eines Wertes prüfen wollen</li><li>✅ Wenn ein Wert als &quot;undefiniert&quot; behandelt werden soll, wenn er nicht gefunden wird.</li><li>✅ Wenn Sie ein bestimmtes Element in einem Array oder einer Liste suchen wollen</li></ul><h3 id="wenn-sie-first-verwenden-sollten" tabindex="-1">Wenn Sie first verwenden sollten <a class="header-anchor" href="#wenn-sie-first-verwenden-sollten" aria-label="Permalink to &quot;Wenn Sie first verwenden sollten&quot;">​</a></h3><ul><li>✅ Wenn Sie den ersten Wert erhalten wollen</li><li>✅ Wenn Sie einen Fehler ausgeben wollen, wenn der Wert nicht gefunden wird</li></ul><h3 id="wann-sollte-filter-verwendet-werden" tabindex="-1">Wann sollte filter verwendet werden? <a class="header-anchor" href="#wann-sollte-filter-verwendet-werden" aria-label="Permalink to &quot;Wann sollte filter verwendet werden?&quot;">​</a></h3><ul><li>✅ Wenn Sie alle Werte benötigen, die eine Bedingung erfüllen</li><li>✅ Wenn Sie die Daten filtern wollen</li></ul><h3 id="anmerkungen" tabindex="-1">Anmerkungen. <a class="header-anchor" href="#anmerkungen" aria-label="Permalink to &quot;Anmerkungen.&quot;">​</a></h3><ul><li>⚠️ <code>find</code> gibt <code>undefined</code> aus, wenn nicht gefunden (kein Fehler)</li><li>⚠️ Wird sofort mit dem ersten Wert abgeschlossen, der die Bedingung erfüllt</li><li>⚠️ TypeScript liefert einen Rückgabewert vom Typ <code>T | undefined</code>.</li></ul><h2 id="🚀-nachster-schritt" tabindex="-1">🚀 Nächster Schritt. <a class="header-anchor" href="#🚀-nachster-schritt" aria-label="Permalink to &quot;🚀 Nächster Schritt.&quot;">​</a></h2><ul><li><strong><a href="./first">first</a></strong> - lernen Sie, wie man den ersten Wert erhält.</li><li><strong><a href="./filter">filter</a></strong> - lernen Sie, wie man auf der Grundlage von Bedingungen filtert.</li><li><strong><a href="https://rxjs.dev/api/operators/findIndex" target="_blank" rel="noreferrer">findIndex</a></strong> - lernen Sie, wie man den Index des ersten Wertes, der eine Bedingung erfüllt, ermittelt (offizielle Dokumentation)</li><li><strong><a href="./practical-use-cases">filtering-operator-practical-use-cases</a></strong> - lernen Sie echte Anwendungsfälle kennen</li></ul>`,47)])])}const c=n(p,[["render",l]]);export{E as __pageData,c as default};
