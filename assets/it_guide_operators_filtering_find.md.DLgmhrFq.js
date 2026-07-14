import{_ as a,o as i,c as n,a2 as p}from"./chunks/framework.BFCpxz1j.js";const c=JSON.parse(`{"title":"find - trova il primo valore che soddisfa la condizione","description":"find è un operatore di filtraggio di RxJS che trova il primo valore che soddisfa una condizione e lo invia in output, completando immediatamente il flusso. È ideale per le situazioni in cui si desidera trovare un elemento specifico da un array o da un elenco, come la ricerca di utenti, il controllo dell'inventario o il rilevamento di log di errore. Se non viene trovato alcun valore, viene emesso undefined e in TypeScript il valore di ritorno è di tipo T | undefined.","frontmatter":{"description":"find è un operatore di filtraggio di RxJS che trova il primo valore che soddisfa una condizione e lo invia in output, completando immediatamente il flusso. È ideale per le situazioni in cui si desidera trovare un elemento specifico da un array o da un elenco, come la ricerca di utenti, il controllo dell'inventario o il rilevamento di log di errore. Se non viene trovato alcun valore, viene emesso undefined e in TypeScript il valore di ritorno è di tipo T | undefined."},"headers":[],"relativePath":"it/guide/operators/filtering/find.md","filePath":"it/guide/operators/filtering/find.md","lastUpdated":1779269732000}`),e={name:"it/guide/operators/filtering/find.md"};function l(t,s,r,h,o,k){return i(),n("div",null,[...s[0]||(s[0]=[p(`<h1 id="find-trova-il-primo-valore-che-soddisfa-la-condizione" tabindex="-1">find - trova il primo valore che soddisfa la condizione <a class="header-anchor" href="#find-trova-il-primo-valore-che-soddisfa-la-condizione" aria-label="Permalink to &quot;find - trova il primo valore che soddisfa la condizione&quot;">​</a></h1><p>L&#39;operatore find trova ed emette il <strong>primo valore che soddisfa la condizione</strong> e completa immediatamente il flusso. Se non viene trovato alcun valore, viene emesso <code>undefined</code>.</p><h2 id="🔰-sintassi-e-uso-di-base" tabindex="-1">🔰 Sintassi e uso di base <a class="header-anchor" href="#🔰-sintassi-e-uso-di-base" aria-label="Permalink to &quot;🔰 Sintassi e uso di base&quot;">​</a></h2><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { from } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { find } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> numbers$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> from</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">([</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">1</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">3</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">5</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">7</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">8</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">9</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">10</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">]);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">numbers$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  find</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">n</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> n </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">%</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 2</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> ===</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 0</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Uscita.: 8(primo numero pari)</span></span></code></pre></div><p><strong>Flusso delle operazioni</strong>:.</p><ol><li>controllo 1, 3, 5, 7 → condizione non soddisfatta</li><li>controllo 8 → condizione soddisfatta → uscita 8 e completa</li><li>9, 10 non valutati</li></ol><p><a href="https://rxjs.dev/api/operators/find" target="_blank" rel="noreferrer">🌐 Documentazione ufficiale di RxJS - find</a></p><h2 id="🆚-contrasto-con-first" tabindex="-1">🆚 Contrasto con first <a class="header-anchor" href="#🆚-contrasto-con-first" aria-label="Permalink to &quot;🆚 Contrasto con first&quot;">​</a></h2><p>Find e first sono simili, ma il loro uso è diverso.</p><p>{\\AN8}CONOSCENZA_2___</p><table tabindex="0"><thead><tr><th>Operatore.</th><th>Specifica della condizione</th><th>Se non viene trovato alcun valore</th><th>Caso d&#39;uso.</th></tr></thead><tbody><tr><td>first()</td><td>Opzione</td><td>Errore (<code>EmptyError</code>)</td><td>Ottenere il primo valore</td></tr><tr><td>first(predicato)</td><td>Opzionale</td><td>Errore (<code>EmptyError</code>)</td><td>Ottenere in modo condizionale.</td></tr><tr><td>find(predicato)</td><td>Richiesto.</td><td>Uscita <code>indefinita</code>.</td><td>Ricerca e controllo dell&#39;esistenza</td></tr></tbody></table><h2 id="💡-modello-di-utilizzo-tipico" tabindex="-1">💡 Modello di utilizzo tipico <a class="header-anchor" href="#💡-modello-di-utilizzo-tipico" aria-label="Permalink to &quot;💡 Modello di utilizzo tipico&quot;">​</a></h2><ol><li><strong>Ricerca dell&#39;utente</strong>.</li></ol><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">   import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { from } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
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
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">   // ID(la condizione è facoltativa)2Ricerca di utenti con</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   users$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">     find</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">user</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> user.id </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">===</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 2</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   ).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">user</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">     if</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> (user) {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">       console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`Trovato: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">user</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">name</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">     } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">else</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">       console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Utente non trovato&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">     }</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   });</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">   // Uscita.: Trovato: Bob</span></span></code></pre></div><ol start="2"><li><p><strong>Controllo dell&#39;inventario</strong></p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { from } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { find } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">interface</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> Product</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">  id</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">:</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> string</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">  name</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">:</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> string</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">  stock</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">:</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> number</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">}</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> products$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> from</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">([</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  { id: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;A1&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, name: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;TaccuinoPC&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, stock: </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">0</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> },</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  { id: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;A2&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, name: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Mouse&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, stock: </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">15</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> },</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  { id: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;A3&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, name: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Tastiere&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, stock: </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">8</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> }</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">] </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">as</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> Product</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">[]);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Scopri cosa è esaurito</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">products$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  find</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">product</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> product.stock </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">===</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 0</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">product</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">  if</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> (product) {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">    console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`Esaurito: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">product</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">name</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">else</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">    console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Tutti in magazzino&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  }</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">});</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Uscita.: Esaurito: TaccuinoPC</span></span></code></pre></div></li><li><p><strong>Ricerca del registro degli errori</strong></p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { from } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
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
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Ricerca del primo errore</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">logs$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  find</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">log</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> log.level </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">===</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;error&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">log</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">  if</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> (log) {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">    console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`Rilevamento dell&#39;errore: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">log</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">message</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">} (Tempo: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">log</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">timestamp</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">})\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  }</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">});</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Uscita.: Rilevamento dell&#39;errore: Connection failed (Tempo: 3)</span></span></code></pre></div></li></ol><h2 id="🧠-esempio-pratico-di-codice-ricerca-di-prodotti" tabindex="-1">🧠 Esempio pratico di codice (ricerca di prodotti) <a class="header-anchor" href="#🧠-esempio-pratico-di-codice-ricerca-di-prodotti" aria-label="Permalink to &quot;🧠 Esempio pratico di codice (ricerca di prodotti)&quot;">​</a></h2><p>Questo è un esempio di ricerca di prodotti corrispondenti a criteri specifici dal magazzino.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { from, fromEvent } da &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { find } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>interfaccia Prodotto {</span></span>
<span class="line"><span>  id: stringa;</span></span>
<span class="line"><span>  nome: stringa</span></span>
<span class="line"><span>  prezzo: numero</span></span>
<span class="line"><span>  categoria: stringa;</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const products: Product[] = [</span></span>
<span class="line"><span>  { id: &#39;P1&#39;, nome: &#39;Mouse wireless&#39;, prezzo: 2980, categoria: &#39;Periferiche PC&#39; }</span></span>
<span class="line"><span>  { id: &#39;P2&#39;, nome: &#39;Tastiera meccanica&#39;, prezzo: 8980, categoria: &#39;Periferiche PC&#39; }</span></span>
<span class="line"><span>  { id: &#39;P3&#39;, nome: &#39;Chiavetta USB 64GB&#39;, prezzo: 1480, categoria: &#39;Archiviazione&#39; }</span></span>
<span class="line"><span>  { id: &#39;P4&#39;, nome: &#39;Monitor 27 pollici&#39;, prezzo: 29800, categoria: &#39;Display&#39; }</span></span>
<span class="line"><span>  { id: &#39;P5&#39;, nome: &#39;Supporto per laptop&#39;, prezzo: 3980, categoria: &#39;Periferiche per PC&#39; }; id: &#39;P5&#39;, nome: &#39;Supporto per laptop&#39;, prezzo: 3980, categoria: &#39;Periferiche per PC&#39; }</span></span>
<span class="line"><span>];</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Creazione di elementi dell&#39;interfaccia utente</span></span>
<span class="line"><span>const container = document.createElement(&#39;div&#39;);.</span></span>
<span class="line"><span>document.body.appendChild(container);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const title = document.createElement(&#39;h3&#39;);</span></span>
<span class="line"><span>title.textContent = &#39;Ricerca prodotti&#39;;</span></span>
<span class="line"><span>container.appendChild(title);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const input = document.createElement(&#39;input&#39;);</span></span>
<span class="line"><span>input.type = &#39;number&#39;;</span></span>
<span class="line"><span>input.placeholder = &#39;Inserisci il prezzo massimo&#39;;</span></span>
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
<span class="line"><span>// Elaborazione della ricerca</span></span>
<span class="line"><span>// Nota: originariamente lo schema consigliato è di appiattire con una switchMap, ma,</span></span>
<span class="line"><span>// Nota: sebbene il modello raccomandato sia quello di appiattire con una switchMap,</span><span> // qui annidiamo il subscribe per la leggibilità,</span><span> // perché include la validazione dell&#39;interfaccia utente (ritorno anticipato).</span></span>
<span class="line"><span>// Considerare un&#39;implementazione piatta, utilizzando switchMap, nel codice di produzione.</span></span>
<span class="line"><span>fromEvent(searchButton, &#39;click&#39;).subscribe(() =&gt; {</span></span>
<span class="line"><span>  const maxPrice = parseInt(input.value);.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>  if (isNaN(maxPrice)) {</span></span>
<span class="line"><span>    result.textContent = &#39;Inserisci un prezzo&#39;;</span></span>
<span class="line"><span>    result.style.color = &#39;red&#39;;</span></span>
<span class="line"><span>    return;</span></span>
<span class="line"><span>  }</span></span>
<span class="line"><span></span></span>
<span class="line"><span>  // Nest subscribe: originariamente si consigliava di appiattire con switchMap</span></span>
<span class="line"><span>  from(prodotti).pipe(</span></span>
<span class="line"><span>    find(product =&gt; product.price &lt;= maxPrice)</span></span>
<span class="line"><span>  ).subscribe(prodotto =&gt; {</span></span>
<span class="line"><span>    if (prodotto) {</span></span>
<span class="line"><span>      result.innerHTML = \`</span></span>
<span class="line"><span>        &lt;strong&gt;Trovato! &lt;/strong&gt;&lt;br&gt;</span></span>
<span class="line"><span>        Nome del prodotto: \${product.name}&lt;br&gt;</span></span>
<span class="line"><span>        Prezzo: \${product.price.toLocaleString()}&lt;br&gt;</span></span>
<span class="line"><span>        Categoria: \${product.category}</span></span>
<span class="line"><span>      \`;</span></span>
<span class="line"><span>      result.style.color = &#39;green&#39;;</span></span>
<span class="line"><span>    } else {</span></span>
<span class="line"><span>      result.textContent = \`¥\${maxPrice.toLocaleString()} o meno prodotto non trovato \`;</span></span>
<span class="line"><span>      result.style.color = &#39;orange&#39;; } else { result.textContent = \`¥\${maxPrice.tolocaleStar()} o meno prodotto non trovato \`; result.style.color = &#39;orange&#39;; }</span></span>
<span class="line"><span>    }</span></span>
<span class="line"><span>  });</span></span>
<span class="line"><span>});</span></span></code></pre></div><p>Questo codice cerca e visualizza il primo prodotto al di sotto del prezzo inserito dall&#39;utente.</p><h2 id="🎯-filter-la-differenza-tra" tabindex="-1">🎯 filter La differenza tra <a class="header-anchor" href="#🎯-filter-la-differenza-tra" aria-label="Permalink to &quot;🎯 filter La differenza tra&quot;">​</a></h2><p><code>find</code> e <code>filter</code> sono utilizzati per scopi diversi.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { from } da &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { find, filter } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// filter: restituisce tutti i valori che corrispondono alla condizione</span></span>
<span class="line"><span>numbers$.pipe(</span></span>
<span class="line"><span>  filter(n =&gt; n &gt; 5)</span></span>
<span class="line"><span>).subscribe({</span></span>
<span class="line"><span>  next: console.log,.</span></span>
<span class="line"><span>  complete: () =&gt; console.log(&#39;filter complete&#39;)</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// Uscita: 7, 8, 9, 10, filtro completato</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// find: produce solo il primo valore che corrisponde alla condizione</span></span>
<span class="line"><span>numbers$.pipe(</span></span>
<span class="line"><span>  find(n =&gt; n &gt; 5)</span></span>
<span class="line"><span>).subscribe({</span></span>
<span class="line"><span>  next: console.log,.</span></span>
<span class="line"><span>  complete: () =&gt; console.log(&#39;find complete&#39;)</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// uscita: 7, find complete</span></span></code></pre></div><table tabindex="0"><thead><tr><th>Operatore</th><th>Numero di uscite</th><th>Tempi di completamento</th><th>Caso d&#39;uso</th></tr></thead><tbody><tr><td><code>filter(predicate)</code></td><td>Tutti i valori che corrispondono alla condizione</td><td>Al completamento del flusso originale</td><td>Raffinamento dei dati</td></tr><tr><td><code>find(predicate)</code></td><td>Solo il primo valore che corrisponde ai criteri</td><td>Immediatamente dopo la scoperta</td><td>Ricerca e controllo dell&#39;esistenza</td></tr></tbody></table><h2 id="📋-utilizzo-sicuro-per-i-tipi" tabindex="-1">📋 Utilizzo sicuro per i tipi <a class="header-anchor" href="#📋-utilizzo-sicuro-per-i-tipi" aria-label="Permalink to &quot;📋 Utilizzo sicuro per i tipi&quot;">​</a></h2><p>TypeScript Questo è un esempio di implementazione type-safe che utilizza i generici in</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { Observable, from } da &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { find } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>interfaccia Task {</span></span>
<span class="line"><span>  id: numero;</span></span>
<span class="line"><span>  titolo: stringa</span></span>
<span class="line"><span>  completato: booleano;</span></span>
<span class="line"><span>  priorità: &#39;alta&#39; | &#39;media&#39; | &#39;bassa&#39;; }</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>function findTaskById(</span></span>
<span class="line"><span>  tasks$: Observable,.</span></span>
<span class="line"><span>  id: numero</span></span>
<span class="line"><span>): Observable | undefined&gt; {</span></span>
<span class="line"><span>  return tasks$.pipe(</span></span>
<span class="line"><span>    find(task =&gt; task.id === id)</span></span>
<span class="line"><span>  );</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>function findFirstIncompleteTask(</span></span>
<span class="line"><span>  tasks$: Observable</span></span>
<span class="line"><span>): Observable | undefined&gt; {</span></span>
<span class="line"><span>  return tasks$.pipe(</span></span>
<span class="line"><span>    find(task =&gt; !task.complete)</span></span>
<span class="line"><span>  );</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Esempio di utilizzo</span></span>
<span class="line"><span>const tasks$ = from([.</span></span>
<span class="line"><span>  { id: 1, titolo: &#39;Compito A&#39;, completato: true, priorità: &#39;alta&#39; as const }</span></span>
<span class="line"><span>  { id: 2, titolo: &#39;Compito B&#39;, completato: false, priorità: &#39;media&#39; as const }</span></span>
<span class="line"><span>  { id: 3, title: &#39;Task C&#39;, completed: false, priority: &#39;low&#39; as const }</span></span>
<span class="line"><span>] come Task[]);.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Ricerca per ID</span></span>
<span class="line"><span>findTaskById(tasks$, 2).subscribe(task =&gt; {</span></span>
<span class="line"><span>  if (task) {</span></span>
<span class="line"><span>    console.log(\`trovato: \${task.title}\`);</span></span>
<span class="line"><span>  } else {</span></span>
<span class="line"><span>    console.log(&#39;Compito non trovato&#39;); }</span></span>
<span class="line"><span>  }</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// Output: trovato: compito B</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Trovare i compiti non completati</span></span>
<span class="line"><span>findFirstIncompleteTask(tasks$).subscribe(task =&gt; {</span></span>
<span class="line"><span>  if (task) {</span></span>
<span class="line"><span>    console.log(\`Prossimo compito: \${task.title} (priorità: \${task.priority})\`);</span></span>
<span class="line"><span>  }</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// Output: prossimo task: task B (priorità: media)</span></span></code></pre></div><h2 id="🔄-find-e-findindex-la-differenza-tra" tabindex="-1">🔄 find e findIndex La differenza tra <a class="header-anchor" href="#🔄-find-e-findindex-la-differenza-tra" aria-label="Permalink to &quot;🔄 find e findIndex La differenza tra&quot;">​</a></h2><p>RxJSnegli operatori <code>findIndex</code> sono disponibili anche gli operatori</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts</span></span>
<span class="line"><span>import { from } da &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { find, findIndex } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const numbers$ = from([10, 20, 30, 40, 50]);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// find: restituisce un valore</span></span>
<span class="line"><span>numbers$.pipe(</span></span>
<span class="line"><span>  find(n =&gt; n &gt; 25)</span></span>
<span class="line"><span>).subscribe(console.log);.</span></span>
<span class="line"><span>// uscita: 30</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// findIndex: restituisce l&#39;indice</span></span>
<span class="line"><span>numbers$.pipe(</span></span>
<span class="line"><span>  findIndex(n =&gt; n &gt; 25)</span></span>
<span class="line"><span>).subscribe(console.log);.</span></span>
<span class="line"><span>// Output: 2 (indice di 30)</span></span></code></pre></div><table tabindex="0"><thead><tr><th>Operatore</th><th>Valore di ritorno</th><th>se il valore non viene trovato</th></tr></thead><tbody><tr><td><code>find(predicate)</code></td><td>Valore stesso</td><td><code>undefined</code></td></tr><tr><td><code>findIndex(predicate)</code></td><td>Indice (valore numerico)</td><td><code>-1</code></td></tr></tbody></table><h2 id="⚠️-errori-comuni" tabindex="-1">⚠️ Errori comuni <a class="header-anchor" href="#⚠️-errori-comuni" aria-label="Permalink to &quot;⚠️ Errori comuni&quot;">​</a></h2><div class="note custom-block github-alert"><p class="custom-block-title">NOTE</p><p><code>find</code> se il valore non viene trovato. <code>undefined</code> viene emesso. Questo non comporta un errore. Se è richiesto un errore, utilizzare <code>first</code> da utilizzare.</p></div><h3 id="errore-gestione-dell-errore-previsto-se-il-valore-non-viene-trovato" tabindex="-1">Errore.: Gestione dell&#39;errore previsto se il valore non viene trovato. <a class="header-anchor" href="#errore-gestione-dell-errore-previsto-se-il-valore-non-viene-trovato" aria-label="Permalink to &quot;Errore.: Gestione dell&#39;errore previsto se il valore non viene trovato.&quot;">​</a></h3><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { from } da &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { find } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const numbers$ = from([1, 3, 5, 7]);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ❌ Cattivo esempio: la gestione degli errori è prevista ma non viene chiamata</span></span>
<span class="line"><span>numbers$.pipe(</span></span>
<span class="line"><span>  find(n =&gt; n &gt; 10)</span></span>
<span class="line"><span>).subscribe({</span></span>
<span class="line"><span>  next: console.log,.</span></span>
<span class="line"><span>  error: err =&gt; console.log(&#39;Error:&#39;, err) // non chiamato</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// uscita: indefinita</span></span></code></pre></div><h3 id="positivo-undefined-controlla-o-first-utilizzare-il-valore" tabindex="-1">Positivo: undefined Controlla o first utilizzare il valore <a class="header-anchor" href="#positivo-undefined-controlla-o-first-utilizzare-il-valore" aria-label="Permalink to &quot;Positivo: undefined Controlla o first utilizzare il valore&quot;">​</a></h3><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>importare { from } da &#39;rxjs&#39;;</span></span>
<span class="line"><span>import { find, first } from &#39;rxjs&#39;;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const numbers$ = from([1, 3, 5, 7]);</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ✅ Buon esempio 1: controllo degli indefiniti</span></span>
<span class="line"><span>numbers$.pipe(</span></span>
<span class="line"><span>  find(n =&gt; n &gt; 10)</span></span>
<span class="line"><span>).subscribe(risultato =&gt; {</span></span>
<span class="line"><span>  if (result ! == undefined) {</span></span>
<span class="line"><span>    console.log(&#39;Trovato:&#39;, risultato);</span></span>
<span class="line"><span>  } else {</span></span>
<span class="line"><span>    console.log(&#39;Non trovato:&#39;); }</span></span>
<span class="line"><span>  }</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// Output: non trovato</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ✅ Buon esempio 2: usare il first se si ha bisogno di un errore</span></span>
<span class="line"><span>numbers$.pipe(</span></span>
<span class="line"><span>  first(n =&gt; n &gt; 10, 0) // specifica il valore predefinito</span></span>
<span class="line"><span>).subscribe({</span></span>
<span class="line"><span>  next: console.log,.</span></span>
<span class="line"><span>  error: err =&gt; console.log(&#39;Error:&#39;, err.message)</span></span>
<span class="line"><span>});</span></span>
<span class="line"><span>// Uscita: 0</span></span></code></pre></div><h2 id="🎓-sommario" tabindex="-1">🎓 Sommario <a class="header-anchor" href="#🎓-sommario" aria-label="Permalink to &quot;🎓 Sommario&quot;">​</a></h2><h3 id="quando-si-dovrebbe-usare-find" tabindex="-1">Quando si dovrebbe usare find. <a class="header-anchor" href="#quando-si-dovrebbe-usare-find" aria-label="Permalink to &quot;Quando si dovrebbe usare find.&quot;">​</a></h3><ul><li>✅ Quando si vuole trovare il primo valore che soddisfa una condizione</li><li>✅ Quando si vuole verificare l&#39;esistenza di un valore</li><li>✅ Quando si vuole trattare un valore come &quot;non definito&quot; se non viene trovato.</li><li>✅ Quando si desidera trovare un elemento specifico in una matrice o in un elenco</li></ul><h3 id="quando-si-dovrebbe-usare-first" tabindex="-1">Quando si dovrebbe usare first <a class="header-anchor" href="#quando-si-dovrebbe-usare-first" aria-label="Permalink to &quot;Quando si dovrebbe usare first&quot;">​</a></h3><ul><li>✅ Se si vuole ottenere il primo valore</li><li>✅ Se si vuole emettere un errore se il valore non è trovato</li></ul><h3 id="quando-si-dovrebbe-usare-il-filtro" tabindex="-1">Quando si dovrebbe usare il filtro? <a class="header-anchor" href="#quando-si-dovrebbe-usare-il-filtro" aria-label="Permalink to &quot;Quando si dovrebbe usare il filtro?&quot;">​</a></h3><ul><li>✅ Se si ha bisogno di tutti i valori che corrispondono a una condizione</li><li>✅ Se si vogliono filtrare i dati</li></ul><h3 id="note" tabindex="-1">Note. <a class="header-anchor" href="#note" aria-label="Permalink to &quot;Note.&quot;">​</a></h3><ul><li>⚠️ <code>find</code> restituisce <code>undefined</code> se non viene trovato (non è un errore).</li><li>⚠️ Completa immediatamente con il primo valore che soddisfa la condizione</li><li>⚠️ TypeScript restituisce un valore di tipo <code>T | undefined</code>.</li></ul><h2 id="🚀-passo-successivo" tabindex="-1">🚀 Passo successivo. <a class="header-anchor" href="#🚀-passo-successivo" aria-label="Permalink to &quot;🚀 Passo successivo.&quot;">​</a></h2><ul><li><strong><a href="./first">first</a></strong> - imparare a ottenere il primo valore.</li><li><strong><a href="./filter">filter</a></strong> - imparare a filtrare in base alle condizioni.</li><li><strong><a href="https://rxjs.dev/api/operators/findIndex" target="_blank" rel="noreferrer">findIndex</a></strong> - per imparare a ottenere l&#39;indice del primo valore che soddisfa una condizione (documentazione ufficiale).</li><li><strong><a href="./practical-use-cases">filtering-operator-practical-use-cases</a></strong> - per imparare casi d&#39;uso reali.</li></ul>`,47)])])}const E=a(e,[["render",l]]);export{c as __pageData,E as default};
