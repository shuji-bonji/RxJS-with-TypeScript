import{_ as i,o as a,c as n,a2 as e}from"./chunks/framework.B0tZAgFO.js";const c=JSON.parse(`{"title":"find - trouver la première valeur qui satisfait la condition","description":"find est un opérateur de filtrage RxJS qui trouve la première valeur qui satisfait une condition et l'affiche, complétant ainsi le flux immédiatement. Il est idéal pour les situations où vous souhaitez trouver un élément spécifique dans un tableau ou une liste, comme la recherche d'utilisateurs, la vérification de l'inventaire ou la détection des journaux d'erreurs. Si aucune valeur n'est trouvée, la sortie est indéfinie et, en TypeScript, la valeur de retour est de type T | indéfini.","frontmatter":{"description":"find est un opérateur de filtrage RxJS qui trouve la première valeur qui satisfait une condition et l'affiche, complétant ainsi le flux immédiatement. Il est idéal pour les situations où vous souhaitez trouver un élément spécifique dans un tableau ou une liste, comme la recherche d'utilisateurs, la vérification de l'inventaire ou la détection des journaux d'erreurs. Si aucune valeur n'est trouvée, la sortie est indéfinie et, en TypeScript, la valeur de retour est de type T | indéfini."},"headers":[],"relativePath":"fr/guide/operators/filtering/find.md","filePath":"fr/guide/operators/filtering/find.md","lastUpdated":1779054497000}`),p={name:"fr/guide/operators/filtering/find.md"};function l(t,s,h,r,k,d){return a(),n("div",null,[...s[0]||(s[0]=[e(`<h1 id="find-trouver-la-premiere-valeur-qui-satisfait-la-condition" tabindex="-1">find - trouver la première valeur qui satisfait la condition <a class="header-anchor" href="#find-trouver-la-premiere-valeur-qui-satisfait-la-condition" aria-label="Permalink to &quot;find - trouver la première valeur qui satisfait la condition&quot;">​</a></h1><p>L&#39;opérateur <code>find</code> trouve et affiche la <strong>première valeur qui satisfait la condition</strong> et termine le flux immédiatement. Si aucune valeur n&#39;est trouvée, il affiche <code>undefined</code>.</p><h2 id="🔰-syntaxe-de-base-et-utilisation" tabindex="-1">🔰 Syntaxe de base et utilisation <a class="header-anchor" href="#🔰-syntaxe-de-base-et-utilisation" aria-label="Permalink to &quot;🔰 Syntaxe de base et utilisation&quot;">​</a></h2><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { from } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { find } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> numbers$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> from</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">([</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">1</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">3</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">5</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">7</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">8</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">9</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">10</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">]);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">numbers$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  find</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">n</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> n </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">%</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 2</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> ===</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 0</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Sortie.: 8(premier nombre pair)</span></span></code></pre></div><p><strong>Flux d&#39;opérations</strong> :.</p><ol><li>vérifier 1, 3, 5, 7 → condition non remplie</li><li>contrôle 8 → condition remplie → sortie 8 et complète</li><li>9, 10 non évalués</li></ol><p><a href="https://rxjs.dev/api/operators/find" target="_blank" rel="noreferrer">🌐 Official RxJS documentation - <code>find</code></a></p><h2 id="🆚-contraste-avec-le-premier" tabindex="-1">🆚 Contraste avec le premier <a class="header-anchor" href="#🆚-contraste-avec-le-premier" aria-label="Permalink to &quot;🆚 Contraste avec le premier&quot;">​</a></h2><p><code>find</code> et <code>first</code> sont similaires, mais leur utilisation est différente.</p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { from } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { find, first } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> numbers$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> from</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">([</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">1</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">3</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">5</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">7</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">8</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">9</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">10</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">]);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// first: Première valeur satisfaisant la condition (la condition est facultative)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">numbers$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  first</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">n</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> n </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">&gt;</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 5</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Sortie.: 7</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// find: Première valeur satisfaisant la condition (la condition est obligatoire)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">numbers$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  find</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">n</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> n </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">&gt;</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 5</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(console.log);</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Sortie.: 7</span></span></code></pre></div><table tabindex="0"><thead><tr><th>Opérateur.</th><th>Spécification de la condition</th><th>Si aucune valeur n&#39;est trouvée</th><th>Cas d&#39;utilisation.</th></tr></thead><tbody><tr><td><code>premier()</code></td><td>Option</td><td>Erreur (<code>EmptyError</code>)</td><td>Obtenir la première valeur</td></tr><tr><td><code>first(predicate)</code></td><td>Option</td><td>Erreur (<code>EmptyError</code>)</td><td>Obtention conditionnelle.</td></tr><tr><td><code>find(predicate)</code></td><td>Obligatoire.</td><td>Sortie <code>undefined</code>.</td><td>Recherche et vérification de l&#39;existence</td></tr></tbody></table><h2 id="💡-modele-d-utilisation-typique" tabindex="-1">💡 Modèle d&#39;utilisation typique <a class="header-anchor" href="#💡-modele-d-utilisation-typique" aria-label="Permalink to &quot;💡 Modèle d&#39;utilisation typique&quot;">​</a></h2><ol><li><strong>Recherche d&#39;un utilisateur</strong>.</li></ol><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">   import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { from } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
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
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">   // ID(la condition est facultative)2Recherche d&#39;utilisateurs ayant</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   users$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">     find</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">user</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> user.id </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">===</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 2</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   ).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">user</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">     if</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> (user) {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">       console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`Trouvé: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">user</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">name</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">     } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">else</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">       console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Utilisateur non trouvé&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">     }</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">   });</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">   // Sortie.: Trouvé: Bob</span></span></code></pre></div><ol start="2"><li><p><strong>Vérification de l&#39;inventaire</strong></p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { from } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { find } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">interface</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> Product</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">  id</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">:</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> string</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">  name</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">:</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> string</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">  stock</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">:</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> number</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">}</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">const</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> products$</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> from</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">([</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  { id: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;A1&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, name: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Ordinateur portablePC&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, stock: </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">0</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> },</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  { id: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;A2&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, name: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Souris&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, stock: </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">15</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> },</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  { id: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;A3&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, name: </span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Claviers&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">, stock: </span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;">8</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> }</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">] </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">as</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;"> Product</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">[]);</span></span>
<span class="line"></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Voir ce qui est en rupture de stock</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">products$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  find</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">product</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> product.stock </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">===</span><span style="--shiki-light:#005CC5;--shiki-dark:#79B8FF;"> 0</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">product</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">  if</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> (product) {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">    console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`En rupture de stock: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">product</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">name</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">}\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">else</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">    console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">&#39;Tous en stock&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  }</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">});</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Sortie.: En rupture de stock: Ordinateur portablePC</span></span></code></pre></div></li><li><p><strong>Rechercher le journal des erreurs</strong></p><div class="language-ts vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang">ts</span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">import</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> { from } </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">from</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;rxjs&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">;</span></span>
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
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Rechercher la première erreur</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">logs$.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">pipe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span></span>
<span class="line"><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">  find</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">log</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> log.level </span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">===</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;"> &#39;error&#39;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">)</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">).</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">subscribe</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#E36209;--shiki-dark:#FFAB70;">log</span><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;"> =&gt;</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> {</span></span>
<span class="line"><span style="--shiki-light:#D73A49;--shiki-dark:#F97583;">  if</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;"> (log) {</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">    console.</span><span style="--shiki-light:#6F42C1;--shiki-dark:#B392F0;">log</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">(</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">\`Détection d&#39;erreur: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">log</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">message</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">} (Temps: \${</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">log</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">.</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">timestamp</span><span style="--shiki-light:#032F62;--shiki-dark:#9ECBFF;">})\`</span><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">);</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">  }</span></span>
<span class="line"><span style="--shiki-light:#24292E;--shiki-dark:#E1E4E8;">});</span></span>
<span class="line"><span style="--shiki-light:#6A737D;--shiki-dark:#6A737D;">// Sortie.: Détection d&#39;erreur: Connection failed (Temps: 3)</span></span></code></pre></div></li></ol><h2 id="🧠-exemple-de-code-pratique-recherche-de-produits" tabindex="-1">🧠 Exemple de code pratique (recherche de produits) <a class="header-anchor" href="#🧠-exemple-de-code-pratique-recherche-de-produits" aria-label="Permalink to &quot;🧠 Exemple de code pratique (recherche de produits)&quot;">​</a></h2><p>Il s&#39;agit d&#39;un exemple de recherche de produits correspondant à des critères spécifiques dans le stock.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { from, fromEvent } from &#39;rxjs&#39; ;</span></span>
<span class="line"><span>import { find } from &#39;rxjs&#39; ;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>interface Product {</span></span>
<span class="line"><span>  id : string ;</span></span>
<span class="line"><span>  name : chaîne de caractères ;</span></span>
<span class="line"><span>  price : nombre ;</span></span>
<span class="line"><span>  category : string ;</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const products : Product[] = [</span></span>
<span class="line"><span>  { id : &#39;P1&#39;, name : &#39;Wireless mouse&#39;, price : 2980, category : &#39;PC peripherals&#39; }</span></span>
<span class="line"><span>  { id : &#39;P2&#39;, name : &#39;Mechanical Keyboard&#39;, price : 8980, category : &#39;PC Peripherals&#39; }</span></span>
<span class="line"><span>  { id : &#39;P3&#39;, name : &#39;Clé USB 64GB&#39;, price : 1480, category : &#39;Storage&#39; }</span></span>
<span class="line"><span>  { id : &#39;P4&#39;, name : &#39;Moniteur 27 pouces&#39;, price : 29800, category : &#39;Displays&#39; }</span></span>
<span class="line"><span>  { id : &#39;P5&#39;, name : &#39;Support d&#39;ordinateur portable&#39;, price : 3980, category : &#39;Périphériques PC&#39; }</span></span>
<span class="line"><span>] ;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Création d&#39;éléments d&#39;interface utilisateur</span></span>
<span class="line"><span>const container = document.createElement(&#39;div&#39;) ;.</span></span>
<span class="line"><span>document.body.appendChild(container) ;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const title = document.createElement(&#39;h3&#39;) ;</span></span>
<span class="line"><span>title.textContent = &quot;Recherche de produits&quot; ;</span></span>
<span class="line"><span>container.appendChild(title) ;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const input = document.createElement(&#39;input&#39;) ;</span></span>
<span class="line"><span>input.type = &#39;number&#39; ;</span></span>
<span class="line"><span>input.placeholder = &quot;Entrez le prix maximum&quot; ;</span></span>
<span class="line"><span>input.style.marginRight = &#39;10px&#39; ;</span></span>
<span class="line"><span>container.appendChild(input) ;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const searchButton = document.createElement(&#39;button&#39;) ;</span></span>
<span class="line"><span>searchButton.textContent = &#39;search&#39; ;</span></span>
<span class="line"><span>container.appendChild(searchButton) ;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const result = document.createElement(&#39;div&#39;) ;</span></span>
<span class="line"><span>result.style.marginTop = &#39;10px&#39; ;</span></span>
<span class="line"><span>container.appendChild(result) ;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Traitement de la recherche</span></span>
<span class="line"><span>// Note : à l&#39;origine, le modèle recommandé est d&#39;aplatir avec un switchMap, mais..,</span></span>
<span class="line"><span>// Note : Bien que le modèle recommandé soit d&#39;aplatir avec un switchMap,</span><span> // ici nous imbriquons le subscribe pour des raisons de lisibilité,</span><span> // parce qu&#39;il inclut la validation de l&#39;interface utilisateur (retour anticipé).</span></span>
<span class="line"><span>// Considérez une implémentation plate utilisant \`switchMap\` dans le code de production.</span></span>
<span class="line"><span>fromEvent(searchButton, &#39;click&#39;).subscribe(() =&gt; {</span></span>
<span class="line"><span>  const maxPrice = parseInt(input.value) ;.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>  if (isNaN(maxPrice)) {</span></span>
<span class="line"><span>    result.textContent = &#39;Veuillez saisir un prix&#39; ;</span></span>
<span class="line"><span>    result.style.colour = &#39;red&#39; ;</span></span>
<span class="line"><span>    return ;</span></span>
<span class="line"><span>  }</span></span>
<span class="line"><span></span></span>
<span class="line"><span>  // Nest subscribe : recommandé à l&#39;origine d&#39;aplatir avec switchMap</span></span>
<span class="line"><span>  from(produits).pipe(</span></span>
<span class="line"><span>    find(produit =&gt; produit.prix &lt;= maxPrice)</span></span>
<span class="line"><span>  ).subscribe(produit =&gt; {</span></span>
<span class="line"><span>    if (product) {</span></span>
<span class="line"><span>      result.innerHTML = \`</span></span>
<span class="line"><span>        &lt;strong&gt;Trouvé ! &lt;/strong&gt;&lt;br&gt;</span></span>
<span class="line"><span>        Nom du produit : \${product.name}&lt;br&gt;</span></span>
<span class="line"><span>        Prix : \${product.price.toLocaleString()}&lt;br&gt;</span></span>
<span class="line"><span>        Catégorie : \${product.category}</span></span>
<span class="line"><span>      \` ;</span></span>
<span class="line"><span>      result.style.color = &#39;green&#39; ;</span></span>
<span class="line"><span>    } else {</span></span>
<span class="line"><span>      result.textContent = \`¥\${maxPrice.toLocaleString()} ou moins produit non trouvé \` ;</span></span>
<span class="line"><span>      result.style.color = &#39;orange&#39; ; }</span></span>
<span class="line"><span>    }</span></span>
<span class="line"><span>  }) ;</span></span>
<span class="line"><span>}) ;</span></span></code></pre></div><p>Ce code recherche et affiche le premier produit dont le prix est inférieur à celui introduit par l&#39;utilisateur.</p><h2 id="🎯-filter-la-difference-entre" tabindex="-1">🎯 filter La différence entre <a class="header-anchor" href="#🎯-filter-la-difference-entre" aria-label="Permalink to &quot;🎯 filter La différence entre&quot;">​</a></h2><p><code>find</code> et <code>filter</code> sont utilisées à des fins différentes.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { from } from &#39;rxjs&#39; ;</span></span>
<span class="line"><span>import { find, filter } from &#39;rxjs&#39; ;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const numbers$ = from([1, 3, 5, 7, 8, 9, 10]) ;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// filtre : sortie de toutes les valeurs qui correspondent à la condition</span></span>
<span class="line"><span>numbers$.pipe(</span></span>
<span class="line"><span>  filter(n =&gt; n &gt; 5)</span></span>
<span class="line"><span>).subscribe({</span></span>
<span class="line"><span>  next : console.log,.</span></span>
<span class="line"><span>  complete : () =&gt; console.log(&#39;filter complete&#39;)</span></span>
<span class="line"><span>}) ;</span></span>
<span class="line"><span>// Sortie : 7, 8, 9, 10, filtre terminé</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// find : sortie uniquement de la première valeur qui correspond à la condition</span></span>
<span class="line"><span>numbers$.pipe(</span></span>
<span class="line"><span>  find(n =&gt; n &gt; 5)</span></span>
<span class="line"><span>).subscribe({</span></span>
<span class="line"><span>  next : console.log,.</span></span>
<span class="line"><span>  complete : () =&gt; console.log(&#39;find complete&#39;)</span></span>
<span class="line"><span>}) ;</span></span>
<span class="line"><span>// résultat : 7, recherche complète</span></span></code></pre></div><table tabindex="0"><thead><tr><th>Opérateur</th><th>Nombre de sorties</th><th>Délai d&#39;exécution</th><th>Cas d&#39;utilisation</th></tr></thead><tbody><tr><td><code>filter(predicate)</code></td><td>Toutes les valeurs correspondant à la condition</td><td>À la fin du flux original</td><td>Affinage des données</td></tr><tr><td><code>find(predicate)</code></td><td>Seule la première valeur correspondant aux critères</td><td>Immédiatement après la découverte</td><td>Recherche et vérification de l&#39;existence</td></tr></tbody></table><h2 id="📋-utilisation-a-securite-de-type" tabindex="-1">📋 Utilisation à sécurité de type <a class="header-anchor" href="#📋-utilisation-a-securite-de-type" aria-label="Permalink to &quot;📋 Utilisation à sécurité de type&quot;">​</a></h2><p>TypeScript Il s&#39;agit d&#39;un exemple d&#39;implémentation à sécurité de type qui utilise les éléments génériques dans le cadre de l&#39;analyse des données.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { Observable, from } from &#39;rxjs&#39; ;</span></span>
<span class="line"><span>import { find } from &#39;rxjs&#39; ;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>interface Task {</span></span>
<span class="line"><span>  id : nombre ;</span></span>
<span class="line"><span>  title : chaîne de caractères ;</span></span>
<span class="line"><span>  completed : booléen ;</span></span>
<span class="line"><span>  priority : &#39;high&#39; | &#39;medium&#39; | &#39;low&#39; ; }</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>function findTaskById(</span></span>
<span class="line"><span>  tasks$ : Observable&lt;Task&gt;,.</span></span>
<span class="line"><span>  id : nombre</span></span>
<span class="line"><span>) : Observable&lt;Task | undefined&gt; {</span></span>
<span class="line"><span>  return tasks$.pipe(</span></span>
<span class="line"><span>    find(task =&gt; task.id === id)</span></span>
<span class="line"><span>  ) ;</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>function findFirstIncompleteTask(</span></span>
<span class="line"><span>  tasks$ : Observable&lt;Task&gt;</span></span>
<span class="line"><span>) : Observable&lt;Task | undefined&gt; {</span></span>
<span class="line"><span>  return tasks$.pipe(</span></span>
<span class="line"><span>    find(task =&gt; !task.completed)</span></span>
<span class="line"><span>  ) ;</span></span>
<span class="line"><span>}</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Exemple d&#39;utilisation</span></span>
<span class="line"><span>const tasks$ = from([.</span></span>
<span class="line"><span>  { id : 1, title : &#39;Tâche A&#39;, completed : true, priority : &#39;high&#39; as const }</span></span>
<span class="line"><span>  { id : 2, title : &#39;Tâche B&#39;, completed : false, priority : &#39;medium&#39; as const }</span></span>
<span class="line"><span>  { id : 3, title : &#39;Tâche C&#39;, completed : false, priority : &#39;low&#39; as const }</span></span>
<span class="line"><span>] en tant que Task[]) ;.</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Recherche par ID</span></span>
<span class="line"><span>findTaskById(tasks$, 2).subscribe(task =&gt; {</span></span>
<span class="line"><span>  if (task) {</span></span>
<span class="line"><span>    console.log(\`trouvé : \${task.title}\`) ;</span></span>
<span class="line"><span>  } else {</span></span>
<span class="line"><span>    console.log(&#39;Task not found&#39;) ; }</span></span>
<span class="line"><span>  }</span></span>
<span class="line"><span>}) ;</span></span>
<span class="line"><span>// Sortie : trouvée : tâche B</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// Recherche des tâches inachevées</span></span>
<span class="line"><span>findFirstIncompleteTask(tasks$).subscribe(task =&gt; {</span></span>
<span class="line"><span>  if (task) {</span></span>
<span class="line"><span>    console.log(\`Tâche suivante : \${task.title} (priorité : \${task.priority})\`) ;</span></span>
<span class="line"><span>  }</span></span>
<span class="line"><span>}) ;</span></span>
<span class="line"><span>// Sortie : tâche suivante : tâche B (priorité : moyenne)</span></span></code></pre></div><h2 id="🔄-find-et-findindex-la-difference-entre" tabindex="-1">🔄 find et findIndex La différence entre <a class="header-anchor" href="#🔄-find-et-findindex-la-difference-entre" aria-label="Permalink to &quot;🔄 find et findIndex La différence entre&quot;">​</a></h2><p>RxJSdans les <code>findIndex</code> sont également disponibles.</p><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts</span></span>
<span class="line"><span>import { from } from &#39;rxjs&#39; ;</span></span>
<span class="line"><span>import { find, findIndex } from &#39;rxjs&#39; ;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const numbers$ = from([10, 20, 30, 40, 50]) ;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// find : renvoie une valeur</span></span>
<span class="line"><span>numbers$.pipe(</span></span>
<span class="line"><span>  find(n =&gt; n &gt; 25)</span></span>
<span class="line"><span>).subscribe(console.log) ;.</span></span>
<span class="line"><span>// sortie : 30</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// findIndex : return index</span></span>
<span class="line"><span>numbers$.pipe(</span></span>
<span class="line"><span>  findIndex(n =&gt; n &gt; 25)</span></span>
<span class="line"><span>).subscribe(console.log) ;.</span></span>
<span class="line"><span>// Résultat : 2 (indice de 30)</span></span></code></pre></div><table tabindex="0"><thead><tr><th>Opérateur</th><th>Retourner la valeur</th><th>si la valeur n&#39;est pas trouvée</th></tr></thead><tbody><tr><td><code>find(predicate)</code></td><td>Valeur elle-même</td><td><code>undefined</code></td></tr><tr><td><code>findIndex(predicate)</code></td><td>Index (valeur numérique)</td><td><code>-1</code></td></tr></tbody></table><h2 id="⚠️-erreurs-courantes" tabindex="-1">⚠️ Erreurs courantes <a class="header-anchor" href="#⚠️-erreurs-courantes" aria-label="Permalink to &quot;⚠️ Erreurs courantes&quot;">​</a></h2><div class="note custom-block github-alert"><p class="custom-block-title">NOTE</p><p><code>find</code> si la valeur n&#39;est pas trouvée. <code>undefined</code> est édité. Cela n&#39;entraîne pas d&#39;erreur. Si une erreur est requise, utilisez <code>first</code> pour être utilisée.</p></div><h3 id="erreur-traitement-de-l-erreur-attendue-si-la-valeur-n-est-pas-trouvee" tabindex="-1">Erreur.: Traitement de l&#39;erreur attendue si la valeur n&#39;est pas trouvée. <a class="header-anchor" href="#erreur-traitement-de-l-erreur-attendue-si-la-valeur-n-est-pas-trouvee" aria-label="Permalink to &quot;Erreur.: Traitement de l&#39;erreur attendue si la valeur n&#39;est pas trouvée.&quot;">​</a></h3><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { from } from &#39;rxjs&#39; ;</span></span>
<span class="line"><span>import { find } from &#39;rxjs&#39; ;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const numbers$ = from([1, 3, 5, 7]) ;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ❌ Mauvais exemple : la gestion des erreurs est attendue mais n&#39;est pas appelée</span></span>
<span class="line"><span>numbers$.pipe(</span></span>
<span class="line"><span>  find(n =&gt; n &gt; 10)</span></span>
<span class="line"><span>).subscribe({</span></span>
<span class="line"><span>  next : console.log,.</span></span>
<span class="line"><span>  error : err =&gt; console.log(&#39;Error:&#39;, err) // pas appelé</span></span>
<span class="line"><span>}) ;</span></span>
<span class="line"><span>// résultat : non défini</span></span></code></pre></div><h3 id="positif-undefined-verifier-ou-first-utiliser-le" tabindex="-1">Positif: undefined Vérifier ou first utiliser le <a class="header-anchor" href="#positif-undefined-verifier-ou-first-utiliser-le" aria-label="Permalink to &quot;Positif: undefined Vérifier ou first utiliser le&quot;">​</a></h3><div class="language- vp-adaptive-theme"><button title="Copy Code" class="copy"></button><span class="lang"></span><pre class="shiki shiki-themes github-light github-dark vp-code" tabindex="0"><code><span class="line"><span></span></span>
<span class="line"><span>ts.</span></span>
<span class="line"><span>import { from } from &#39;rxjs&#39; ;</span></span>
<span class="line"><span>import { find, first } from &#39;rxjs&#39; ;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>const numbers$ = from([1, 3, 5, 7]) ;</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ✅ Bon exemple 1 : vérification de l&#39;absence de définition</span></span>
<span class="line"><span>numbers$.pipe(</span></span>
<span class="line"><span>  find(n =&gt; n &gt; 10)</span></span>
<span class="line"><span>).subscribe(result =&gt; {</span></span>
<span class="line"><span>  if (result ! == undefined) {</span></span>
<span class="line"><span>    console.log(&#39;Found:&#39;, result) ;</span></span>
<span class="line"><span>  } else {</span></span>
<span class="line"><span>    console.log(&#39;Not found:&#39;) ; }</span></span>
<span class="line"><span>  }</span></span>
<span class="line"><span>}) ;</span></span>
<span class="line"><span>// Résultat : non trouvé</span></span>
<span class="line"><span></span></span>
<span class="line"><span>// ✅ Bon exemple 2 : utilisez le premier si vous avez besoin d&#39;une erreur</span></span>
<span class="line"><span>numbers$.pipe(</span></span>
<span class="line"><span>  first(n =&gt; n &gt; 10, 0) // spécifie la valeur par défaut</span></span>
<span class="line"><span>).subscribe({</span></span>
<span class="line"><span>  next : console.log,.</span></span>
<span class="line"><span>  error : err =&gt; console.log(&#39;Error:&#39;, err.message)</span></span>
<span class="line"><span>}) ;</span></span>
<span class="line"><span>// Sortie : 0</span></span></code></pre></div><h2 id="🎓-resume" tabindex="-1">🎓 Résumé <a class="header-anchor" href="#🎓-resume" aria-label="Permalink to &quot;🎓 Résumé&quot;">​</a></h2><h3 id="quand-utiliser-find" tabindex="-1">Quand utiliser find. <a class="header-anchor" href="#quand-utiliser-find" aria-label="Permalink to &quot;Quand utiliser find.&quot;">​</a></h3><ul><li>✅ Lorsque vous voulez trouver la première valeur qui satisfait une condition.</li><li>✅ Lorsque vous voulez vérifier l&#39;existence d&#39;une valeur</li><li>✅ Lorsque vous voulez traiter une valeur comme <code>undefined</code> si elle n&#39;est pas trouvée.</li><li>✅ Lorsque vous voulez trouver un élément spécifique dans un tableau ou une liste</li></ul><h3 id="quand-vous-devez-utiliser-en-premier" tabindex="-1">Quand vous devez utiliser en premier <a class="header-anchor" href="#quand-vous-devez-utiliser-en-premier" aria-label="Permalink to &quot;Quand vous devez utiliser en premier&quot;">​</a></h3><ul><li>✅ Si vous voulez obtenir la première valeur</li><li>✅ Si vous voulez afficher une erreur si la valeur n&#39;est pas trouvée</li></ul><h3 id="quand-faut-il-utiliser-le-filtre" tabindex="-1">Quand faut-il utiliser le filtre ? <a class="header-anchor" href="#quand-faut-il-utiliser-le-filtre" aria-label="Permalink to &quot;Quand faut-il utiliser le filtre ?&quot;">​</a></h3><ul><li>✅ Si vous avez besoin de toutes les valeurs correspondant à une condition</li><li>✅ Si vous voulez filtrer les données</li></ul><h3 id="notes" tabindex="-1">Notes. <a class="header-anchor" href="#notes" aria-label="Permalink to &quot;Notes.&quot;">​</a></h3><ul><li>⚠️ <code>find</code> produit <code>undefined</code> s&#39;il n&#39;est pas trouvé (ce n&#39;est pas une erreur)</li><li>⚠️ Complète immédiatement avec la première valeur qui satisfait la condition</li><li>⚠️ TypeScript donne une valeur de retour de type <code>T | undefined</code>.</li></ul><h2 id="🚀-prochaine-etape" tabindex="-1">🚀 Prochaine étape. <a class="header-anchor" href="#🚀-prochaine-etape" aria-label="Permalink to &quot;🚀 Prochaine étape.&quot;">​</a></h2><ul><li><strong>[first](. /first)</strong> - apprendre à obtenir la première valeur.</li><li><strong>[filter](. /filter)</strong> - Apprenez à filtrer sur la base de conditions.</li><li><strong><a href="https://rxjs.dev/api/operators/findIndex" target="_blank" rel="noreferrer">findIndex</a></strong> - Apprenez à obtenir l&#39;index de la première valeur qui satisfait une condition (documentation officielle).</li><li><strong>[filtering-operator-practical-use-cases](. /practical-use-cases)</strong> - apprendre des cas d&#39;utilisation réels</li></ul>`,47)])])}const E=i(p,[["render",l]]);export{c as __pageData,E as default};
