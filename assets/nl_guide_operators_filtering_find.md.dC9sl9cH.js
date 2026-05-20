import{_ as a,o as n,c as i,a2 as e}from"./chunks/framework.B0tZAgFO.js";const E=JSON.parse('{"title":"find - vind de eerste waarde die aan de voorwaarde voldoet","description":"find is een RxJS filteroperator die de eerste waarde vindt die aan een voorwaarde voldoet en deze uitvoert, waardoor de stroom onmiddellijk wordt voltooid. Het is ideaal voor situaties waarin je een specifiek element uit een array of lijst wilt vinden, zoals het zoeken naar gebruikers, het controleren van de inventaris of het opsporen van foutlogs. Als er geen waarde wordt gevonden, wordt er een ongedefinieerde uitvoer gegeven en in TypeScript is de retourwaarde van het type T | ongedefinieerd.","frontmatter":{"description":"find is een RxJS filteroperator die de eerste waarde vindt die aan een voorwaarde voldoet en deze uitvoert, waardoor de stroom onmiddellijk wordt voltooid. Het is ideaal voor situaties waarin je een specifiek element uit een array of lijst wilt vinden, zoals het zoeken naar gebruikers, het controleren van de inventaris of het opsporen van foutlogs. Als er geen waarde wordt gevonden, wordt er een ongedefinieerde uitvoer gegeven en in TypeScript is de retourwaarde van het type T | ongedefinieerd."},"headers":[],"relativePath":"nl/guide/operators/filtering/find.md","filePath":"nl/guide/operators/filtering/find.md","lastUpdated":1779269732000}'),p={name:"nl/guide/operators/filtering/find.md"};function l(t,s,h,k,r,d){return n(),i("div",null,[...s[0]||(s[0]=[e(`<h1 id="find-vind-de-eerste-waarde-die-aan-de-voorwaarde-voldoet" tabindex="-1">find - vind de eerste waarde die aan de voorwaarde voldoet <a class="header-anchor" href="#find-vind-de-eerste-waarde-die-aan-de-voorwaarde-voldoet" aria-label="Permalink to &quot;find - vind de eerste waarde die aan de voorwaarde voldoet&quot;">​</a></h1><p>De <code>find</code> operator vindt en voert de <strong>eerste waarde uit die aan de voorwaarde voldoet</strong> en voltooit de stroom onmiddellijk. Als er geen waarde wordt gevonden, wordt <code>undefined</code> uitgevoerd.</p><h2 id="🔰-basis-syntaxis-en-gebruik" tabindex="-1">🔰 Basis syntaxis en gebruik <a class="header-anchor" href="#🔰-basis-syntaxis-en-gebruik" aria-label="Permalink to &quot;🔰 Basis syntaxis en gebruik&quot;">​</a></h2><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { from } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { find } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> numbers$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> from</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">([</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">1</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">3</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">5</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">7</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">8</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">9</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">10</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">]);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">numbers$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  find</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">n</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> n </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">%</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 2</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> ===</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 0</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Uitgang.: 8(eerste even getal)</span></span></code></pre></div><p><strong>Bewerkingsstroom</strong>:.</p><ol><li>controleer 1, 3, 5, 7 → voorwaarde niet vervuld</li><li>controleer 8 → voorwaarde vervuld → uitvoer 8 en compleet</li><li>9, 10 niet geëvalueerd</li></ol><p><a href="https://rxjs.dev/api/operators/find" target="_blank" rel="noreferrer">🌐 Officiële RxJS documentatie - find</a></p><h2 id="🆚-contrast-met-first" tabindex="-1">🆚 Contrast met first <a class="header-anchor" href="#🆚-contrast-met-first" aria-label="Permalink to &quot;🆚 Contrast met first&quot;">​</a></h2><p><code>find</code> en <code>first</code> lijken op elkaar, maar ze worden anders gebruikt.</p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { from } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { find, first } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> numbers$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> from</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">([</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">1</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">3</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">5</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">7</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">8</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">9</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">10</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">]);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// first: Eerste waarde die aan de voorwaarde voldoet (voorwaarde is optioneel)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">numbers$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  first</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">n</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> n </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">&gt;</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 5</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Uitgang.: 7</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// find: Eerste waarde die aan de voorwaarde voldoet (voorwaarde is verplicht)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">numbers$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  find</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">n</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> n </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">&gt;</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 5</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Uitgang.: 7</span></span></code></pre></div><p>TABEL 9</p><h2 id="💡-typisch-gebruikspatroon" tabindex="-1">💡 Typisch gebruikspatroon <a class="header-anchor" href="#💡-typisch-gebruikspatroon" aria-label="Permalink to &quot;💡 Typisch gebruikspatroon&quot;">​</a></h2><ol><li><strong>Gebruiker zoeken</strong>.</li></ol><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">   import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { from } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
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
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">   // ID(voorwaarde is optioneel)2Zoeken naar gebruikers met</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   users$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">     find</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">user</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> user.id </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">===</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 2</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   ).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">user</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">     if</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> (user) {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">       console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`Gevonden: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">user</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">name</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">     } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">else</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">       console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Gebruiker niet gevonden&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">     }</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   });</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">   // Uitgang.: Gevonden: Bob</span></span></code></pre></div><ol start="2"><li><p><strong>Inventaris controleren</strong></p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { from } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
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
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  { id: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;A2&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, name: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Muis&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, stock: </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">15</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> },</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  { id: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;A3&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, name: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Toetsenborden&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, stock: </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">8</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> }</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">] </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">as</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> Product</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">[]);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Zoek uit wat niet op voorraad is</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">products$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  find</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">product</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> product.stock </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">===</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 0</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">product</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">  if</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> (product) {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">    console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`Niet op voorraad: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">product</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">name</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">else</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">    console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Alles op voorraad&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  }</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">});</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Uitgang.: Niet op voorraad: NotebookPC</span></span></code></pre></div></li><li><p><strong>Zoeken in foutenlogboek</strong></p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { from } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
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
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Eerste fout zoeken</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">logs$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  find</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">log</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> log.level </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">===</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;error&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">log</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">  if</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> (log) {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">    console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`Foutdetectie: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">log</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">message</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">} (Tijd: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">log</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">timestamp</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">})\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  }</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">});</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Uitgang.: Foutdetectie: Connection failed (Tijd: 3)</span></span></code></pre></div></li></ol><h2 id="🧠-praktijkvoorbeeld-code-product-zoeken" tabindex="-1">🧠 Praktijkvoorbeeld code (product zoeken) <a class="header-anchor" href="#🧠-praktijkvoorbeeld-code-product-zoeken" aria-label="Permalink to &quot;🧠 Praktijkvoorbeeld code (product zoeken)&quot;">​</a></h2><p>Dit is een voorbeeld van het zoeken naar producten uit de voorraad die voldoen aan specifieke criteria.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { from, fromEvent } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { find } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>interface Product {</span></span>
<span class="line"><span>  id: string;</span></span>
<span class="line"><span>  naam: string;</span></span>
<span class="line"><span>  prijs: getal;</span></span>
<span class="line"><span>  categorie: string;</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const products: Product[] = [</span></span>
<span class="line"><span>  { id: &#39;P1&#39;, naam: &#39;Draadloze muis&#39;, prijs: 2980, categorie: &#39;PC-randapparatuur&#39; }</span></span>
<span class="line"><span>  { id: &#39;P2&#39;, naam: &#39;Mechanisch toetsenbord&#39;, prijs: 8980, categorie: &#39;PC-randapparatuur&#39; }</span></span>
<span class="line"><span>  { id: &#39;P3&#39;, naam: &#39;USB-geheugenstick 64GB&#39;, prijs: 1480, categorie: &#39;Opslag&#39; }</span></span>
<span class="line"><span>  { id: &#39;P4&#39;, naam: &#39;Monitor 27-inch&#39;, prijs: 29800, categorie: &#39;Displays&#39; }</span></span>
<span class="line"><span>  { id: &#39;P5&#39;, naam: &#39;laptopstandaard&#39;, prijs: 3980, categorie: &#39;PC-randapparatuur&#39; }</span></span>
<span class="line"><span>];</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// UI-elementen maken</span></span>
<span class="line"><span>const container = document.createElement(&#39;div&#39;);.</span></span>
<span class="line"><span>document.body.appendChild(container);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const titel = document.createElement(&#39;h3&#39;);</span></span>
<span class="line"><span>title.textContent = &quot;Product zoeken&quot;;</span></span>
<span class="line"><span>container.appendChild(title);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const input = document.createElement(&#39;input&#39;);</span></span>
<span class="line"><span>input.type = &#39;getal&#39;;</span></span>
<span class="line"><span>input.placeholder = &quot;Maximale prijs invoeren&quot;;</span></span>
<span class="line"><span>input.style.marginRight = &quot;10px&quot;;</span></span>
<span class="line"><span>container.appendChild(input);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const zoekknop = document.createElement(&#39;knop&#39;);</span></span>
<span class="line"><span>searchButton.textContent = &#39;search&#39;;</span></span>
<span class="line"><span>container.appendChild(searchButton);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const resultaat = document.createElement(&#39;div&#39;);</span></span>
<span class="line"><span>result.style.marginTop = &#39;10px&#39;;</span></span>
<span class="line"><span>container.appendChild(result);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Verwerking zoeken</span></span>
<span class="line"><span>// Opmerking: hoewel het aanbevolen patroon oorspronkelijk is om af te vlakken met een switchMap, maar,</span></span>
<span class="line"><span>// Opmerking: Hoewel het aanbevolen patroon is om af te vlakken met een switchMap,</span><span> // nestelen we hier de subscribe voor de leesbaarheid,</span><span> // omdat het UI-validatie bevat (vroege terugkeer).</span></span>
<span class="line"><span>// Overweeg een platte implementatie met gebruik van \`switchMap\` in productiecode.</span></span>
<span class="line"><span>fromEvent(searchButton, &#39;click&#39;).subscribe() =&gt; {</span></span>
<span class="line"><span>  const maxPrice = parseInt(input.value);.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>  Als (isNaN(maxPrice)) {</span></span>
<span class="line"><span>    result.textContent = &#39;Voer een prijs in&#39;;</span></span>
<span class="line"><span>    resultaat.style.kleur = &#39;rood&#39;;</span></span>
<span class="line"><span>    return;</span></span>
<span class="line"><span>  }</span></span>
<span class="line"><span></span></span>
<span class="line"><span>  // Nest subscribe: oorspronkelijk aanbevolen om af te vlakken met switchMap</span></span>
<span class="line"><span>  from(products).pipe(</span></span>
<span class="line"><span>    find(product =&gt; product.price &lt;= maxPrice)</span></span>
<span class="line"><span>  ).subscribe(product =&gt; {</span></span>
<span class="line"><span>    if (product) {</span></span>
<span class="line"><span>      result.innerHTML = \`</span></span>
<span class="line"><span>        &lt;strong&gt;Gevonden! &lt;/strong&gt;&lt;br&gt;</span></span>
<span class="line"><span>        Productnaam: \${product.name}&lt;br&gt;</span></span>
<span class="line"><span>        Prijs: \${product.price.toLocaleString()}&lt;br&gt;</span></span>
<span class="line"><span>        Categorie: \${product.category}</span></span>
<span class="line"><span>      \`;</span></span>
<span class="line"><span>      resultaat.style.kleur = &#39;groen&#39;;</span></span>
<span class="line"><span>    } anders {</span></span>
<span class="line"><span>      result.textContent = \`¥\${maxPrice.toLocaleString()} of minder product niet gevonden \`;</span></span>
<span class="line"><span>      resultaat.style.kleur = &#39;oranje&#39;; }</span></span>
<span class="line"><span>    }</span></span>
<span class="line"><span>  });</span></span>
<span class="line"><span>});</span></span></code></pre></div><p>Deze code zoekt en toont het eerste product onder de door de gebruiker ingevoerde prijs.</p><h2 id="🎯-filter-het-verschil-tussen" tabindex="-1">🎯 filter Het verschil tussen <a class="header-anchor" href="#🎯-filter-het-verschil-tussen" aria-label="Permalink to &quot;🎯 filter Het verschil tussen&quot;">​</a></h2><p><code>find</code> en <code>filter</code> worden voor verschillende doeleinden gebruikt.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { from } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { find, filter } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const. nummers$ = from([1, 3, 5, 7, 8, 9, 10]);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// filter: uitvoer van alle waarden die aan de voorwaarde voldoen</span></span>
<span class="line"><span>getallen$.pipe(</span></span>
<span class="line"><span>  filter(n =&gt; n &gt; 5)</span></span>
<span class="line"><span>).subscribe({</span></span>
<span class="line"><span>  next: console.log,.</span></span>
<span class="line"><span>  complete: () =&gt; console.log(&#39;filter compleet&#39;)</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// Uitvoer: 7, 8, 9, 10, filter voltooid</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// find: voer alleen de eerste waarde uit die overeenkomt met de voorwaarde</span></span>
<span class="line"><span>getallen$.pipe(</span></span>
<span class="line"><span>  find(n =&gt; n &gt; 5)</span></span>
<span class="line"><span>).subscribe({</span></span>
<span class="line"><span>  next: console.log,.</span></span>
<span class="line"><span>  complete: () =&gt; console.log(&#39;find compleet&#39;)</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// uitvoer: 7, find voltooid</span></span></code></pre></div><table tabindex="0"><thead><tr><th>Operator</th><th>Aantal uitgangen</th><th>Tijdstip van voltooiing</th><th>Gebruik</th></tr></thead><tbody><tr><td><code>filter(predicate)</code></td><td>Alle waarden die aan de voorwaarde voldoen</td><td>Bij voltooiing van oorspronkelijke stroom</td><td>Verfijning van gegevens</td></tr><tr><td><code>find(predicate)</code></td><td>Alleen de eerste waarde die aan de criteria voldoet</td><td>Onmiddellijk na ontdekking</td><td>Zoeken en bestaanscontrole</td></tr></tbody></table><h2 id="📋-type-veilig-gebruik" tabindex="-1">📋 Type-veilig gebruik <a class="header-anchor" href="#📋-type-veilig-gebruik" aria-label="Permalink to &quot;📋 Type-veilig gebruik&quot;">​</a></h2><p>TypeScript Dit is een voorbeeld van een typeveilige implementatie die gebruik maakt van generics in</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { Observable, from } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { find } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>interface Taak {</span></span>
<span class="line"><span>  id: getal;</span></span>
<span class="line"><span>  title: string;</span></span>
<span class="line"><span>  complete: booleaans;</span></span>
<span class="line"><span>  priority: &#39;high&#39; | &#39;medium&#39; | &#39;low&#39;; }</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>functie findTaskById(</span></span>
<span class="line"><span>  taken$: Observable,.</span></span>
<span class="line"><span>  id: getal</span></span>
<span class="line"><span>): Observable | undefined&gt; {</span></span>
<span class="line"><span>  return taken$.pipe(</span></span>
<span class="line"><span>    find(task =&gt; task.id === id)</span></span>
<span class="line"><span>  );</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>functie findFirstIncompleteTask(</span></span>
<span class="line"><span>  taken$: Observable</span></span>
<span class="line"><span>): Observable | undefined&gt; {</span></span>
<span class="line"><span>  return tasks$.pipe(</span></span>
<span class="line"><span>    find(task =&gt; !.task.complete)</span></span>
<span class="line"><span>  );</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Gebruiksvoorbeeld</span></span>
<span class="line"><span>const tasks$ = from([.</span></span>
<span class="line"><span>  { id: 1, titel: &#39;Taak A&#39;, compleet: true, prioriteit: &#39;hoog&#39; als const }</span></span>
<span class="line"><span>  { id: 2, titel: &#39;Taak B&#39;, compleet: false, prioriteit: &#39;medium&#39; als const }</span></span>
<span class="line"><span>  { id: 3, titel: &#39;Taak C&#39;, compleet: false, prioriteit: &#39;laag&#39; als const }.</span></span>
<span class="line"><span>] als Taak[]);.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Zoeken op ID</span></span>
<span class="line"><span>findTaskById(tasks$, 2).subscribe(task =&gt; {</span></span>
<span class="line"><span>  if (taak) {</span></span>
<span class="line"><span>    console.log(\`gevonden: \${task.title}\`);</span></span>
<span class="line"><span>  } anders {</span></span>
<span class="line"><span>    console.log(&#39;Taak niet gevonden&#39;); }</span></span>
<span class="line"><span>  }</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// Uitvoer: gevonden: taak B</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Onvoltooide taken vinden</span></span>
<span class="line"><span>findFirstIncompleteTask(tasks$).subscribe(task =&gt; {</span></span>
<span class="line"><span>  if (taak) {</span></span>
<span class="line"><span>    console.log(\`Volgende taak: \${task.title} (prioriteit: \${task.priority})\`);</span></span>
<span class="line"><span>  }</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// Uitvoer: volgende taak: taak B (prioriteit: gemiddeld)</span></span></code></pre></div><h2 id="🔄-find-en-findindex-het-verschil-tussen" tabindex="-1">🔄 find en findIndex Het verschil tussen <a class="header-anchor" href="#🔄-find-en-findindex-het-verschil-tussen" aria-label="Permalink to &quot;🔄 find en findIndex Het verschil tussen&quot;">​</a></h2><p>RxJSin de <code>findIndex</code> operatoren zijn ook beschikbaar.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts</span></span>
<span class="line"><span>import { from } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { find, findIndex } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const. nummers$ = from([10, 20, 30, 40, 50]);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// find: retourneer een waarde</span></span>
<span class="line"><span>getallen$.pipe(</span></span>
<span class="line"><span>  find(n =&gt; n &gt; 25)</span></span>
<span class="line"><span>).subscribe(console.log);.</span></span>
<span class="line"><span>// uitvoer: 30</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// findIndex: index teruggeven</span></span>
<span class="line"><span>getallen$.pipe(</span></span>
<span class="line"><span>  findIndex(n =&gt; n &gt; 25)</span></span>
<span class="line"><span>).subscribe(console.log);.</span></span>
<span class="line"><span>// Uitvoer: 2 (index van 30)</span></span></code></pre></div><table tabindex="0"><thead><tr><th>Operator</th><th>Waarde retourneren</th><th>als de waarde niet is gevonden</th></tr></thead><tbody><tr><td><code>find(predicate)</code></td><td>Waarde zelf</td><td><code>undefined</code></td></tr><tr><td><code>findIndex(predicate)</code></td><td>Index (numerieke waarde)</td><td><code>-1</code></td></tr></tbody></table><h2 id="⚠️-een-veelgemaakte-fout" tabindex="-1">⚠️ Een veelgemaakte fout <a class="header-anchor" href="#⚠️-een-veelgemaakte-fout" aria-label="Permalink to &quot;⚠️ Een veelgemaakte fout&quot;">​</a></h2><div class="note custom-block github-alert"><p class="custom-block-title">NOTE</p><p><code>find</code> als de waarde niet wordt gevonden. <code>undefined</code> wordt uitgevoerd. Dit resulteert niet in een fout. Als een fout vereist is, moet <code>first</code> worden gebruikt.</p></div><h3 id="fout-verwachte-foutafhandeling-als-de-waarde-niet-wordt-gevonden" tabindex="-1">Fout.: Verwachte foutafhandeling als de waarde niet wordt gevonden. <a class="header-anchor" href="#fout-verwachte-foutafhandeling-als-de-waarde-niet-wordt-gevonden" aria-label="Permalink to &quot;Fout.: Verwachte foutafhandeling als de waarde niet wordt gevonden.&quot;">​</a></h3><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { from } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { find } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const. nummers$ = from([1, 3, 5, 7]);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ❌ Slecht voorbeeld: foutafhandeling verwacht maar niet aangeroepen</span></span>
<span class="line"><span>getallen$.pipe(</span></span>
<span class="line"><span>  find(n =&gt; n &gt; 10)</span></span>
<span class="line"><span>).subscribe({</span></span>
<span class="line"><span>  next: console.log,.</span></span>
<span class="line"><span>  error: err =&gt; console.log(&#39;Error:&#39;, err) // niet aangeroepen</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// uitvoer: ongedefinieerd</span></span></code></pre></div><h3 id="positief-undefined-controleer-of-first-gebruik-de" tabindex="-1">Positief: undefined Controleer of first gebruik de <a class="header-anchor" href="#positief-undefined-controleer-of-first-gebruik-de" aria-label="Permalink to &quot;Positief: undefined Controleer of first gebruik de&quot;">​</a></h3><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { from } from &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { find, first } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const. nummers$ = from([1, 3, 5, 7]);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ✅ Goed voorbeeld 1: controleer op ongedefinieerd</span></span>
<span class="line"><span>getallen$.pipe(</span></span>
<span class="line"><span>  find(n =&gt; n &gt; 10)</span></span>
<span class="line"><span>).subscribe(resultaat =&gt; {</span></span>
<span class="line"><span>  als (resultaat ! == undefined) {</span></span>
<span class="line"><span>    console.log(&#39;Gevonden:&#39;, resultaat);</span></span>
<span class="line"><span>  } anders {</span></span>
<span class="line"><span>    console.log(&#39;Niet gevonden:&#39;); }</span></span>
<span class="line"><span>  }</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// Uitvoer: niet gevonden</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ✅ Goed voorbeeld 2: gebruik first als je een foutmelding nodig hebt</span></span>
<span class="line"><span>getallen$.pipe(</span></span>
<span class="line"><span>  first(n =&gt; n &gt; 10, 0) // standaardwaarde opgeven</span></span>
<span class="line"><span>).subscribe({</span></span>
<span class="line"><span>  next: console.log,.</span></span>
<span class="line"><span>  error: err =&gt; console.log(&#39;Error:&#39;, err.message)</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// Uitvoer: 0</span></span></code></pre></div><h2 id="samenvatting" tabindex="-1">Samenvatting <a class="header-anchor" href="#samenvatting" aria-label="Permalink to &quot;Samenvatting&quot;">​</a></h2><h3 id="wanneer-je-find-moet-gebruiken" tabindex="-1">Wanneer je find moet gebruiken. <a class="header-anchor" href="#wanneer-je-find-moet-gebruiken" aria-label="Permalink to &quot;Wanneer je find moet gebruiken.&quot;">​</a></h3><ul><li>✅ Als je de eerste waarde wilt vinden die aan een voorwaarde voldoet</li><li>✅ Als je het bestaan van een waarde wilt controleren</li><li>✅ Wanneer je een waarde als <code>undefined</code> wilt behandelen als deze niet wordt gevonden.</li><li>✅ Wanneer je een specifiek element in een array of lijst wilt vinden</li></ul><h3 id="wanneer-je-first-moet-gebruiken" tabindex="-1">Wanneer je first moet gebruiken <a class="header-anchor" href="#wanneer-je-first-moet-gebruiken" aria-label="Permalink to &quot;Wanneer je first moet gebruiken&quot;">​</a></h3><ul><li>✅ Als je de eerste waarde wilt krijgen</li><li>✅ Als je een foutmelding wilt geven als de waarde niet wordt gevonden</li></ul><h3 id="wanneer-moet-filter-worden-gebruikt" tabindex="-1">Wanneer moet filter worden gebruikt? <a class="header-anchor" href="#wanneer-moet-filter-worden-gebruikt" aria-label="Permalink to &quot;Wanneer moet filter worden gebruikt?&quot;">​</a></h3><ul><li>✅ Als u alle waarden nodig hebt die aan een voorwaarde voldoen</li><li>✅ Als u de gegevens wilt filteren</li></ul><h3 id="opmerkingen" tabindex="-1">Opmerkingen. <a class="header-anchor" href="#opmerkingen" aria-label="Permalink to &quot;Opmerkingen.&quot;">​</a></h3><ul><li>⚠️ <code>find</code> geeft <code>undefined</code> als het niet gevonden wordt (geen fout)</li><li>⚠️ Beëindigt onmiddellijk met de eerste waarde die aan de voorwaarde voldoet</li><li>⚠️ TypeScript geeft een retourwaarde van het type <code>T | undefined</code>.</li></ul><h2 id="volgende-stap" tabindex="-1">Volgende stap. <a class="header-anchor" href="#volgende-stap" aria-label="Permalink to &quot;Volgende stap.&quot;">​</a></h2><ul><li><strong><a href="./first">first</a></strong> - leer hoe je de eerste waarde krijgt.</li><li><strong><a href="./filter">filter</a></strong> - leer filteren op basis van voorwaarden.</li><li><strong><a href="https://rxjs.dev/api/operators/findIndex" target="_blank" rel="noreferrer">findIndex</a></strong> - leer hoe je de index krijgt van de eerste waarde die voldoet aan een voorwaarde (officiële documentatie)</li><li><strong><a href="./practical-use-cases">filtering-operator-praktische-gebruik-cases</a></strong> - leer echte gebruikssituaties</li></ul>`,47)])])}const g=a(p,[["render",l]]);export{E as __pageData,g as default};
