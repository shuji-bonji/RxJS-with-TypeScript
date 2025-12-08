---
description: "La fonction de création forkJoin produit la dernière valeur de chaque Observable sous forme de tableau ou d'objet une fois que tous les Observables ont été complétés. Idéal lorsque plusieurs requêtes API sont exécutées en parallèle et que tous les résultats sont disponibles avant le traitement."
---

# forkJoin - produire toutes les dernières valeurs ensemble

`forkJoin` est une fonction de création qui produit la dernière valeur de chaque Observable sous la forme d'un tableau ou d'un objet après que **tous** les Observables aient été complétés.
C'est très utile lorsque vous voulez utiliser tous les Observables en même temps.


## Syntaxe de base et utilisation

```ts
import { forkJoin, of } from 'rxjs';
import { delay } from 'rxjs';

const user$ = of('Utilisateur A').pipe(delay(1000));
const posts$ = of('Liste des posts').pipe(delay(1500));

forkJoin([user$, posts$]).subscribe(([user, posts]) => {
  console.log(user, posts);
});

// Sortie:
// Utilisateur A Liste des posts
```

- Attend que tous les Observables soient `complete`.
- Seule la **dernière valeur émise** de chaque Observable est compilée et produite.

[🌐 Documentation officielle RxJS - `forkJoin`](https://rxjs.dev/api/index/function/forkJoin)


## Modèles d'utilisation typiques

- **Exécuter plusieurs requêtes API en parallèle et résumer tous les résultats**
- **Obtenir plusieurs ensembles de données nécessaires pour le chargement initial en une seule fois**
- **Obtenir toutes les données pertinentes en une seule fois et effectuer le rendu de l'écran en une seule fois**


## Exemples de code pratique (avec interface utilisateur)

Simuler plusieurs requêtes API et les afficher ensemble lorsque tous les résultats sont disponibles.

```ts
import { forkJoin, of } from 'rxjs';
import { delay } from 'rxjs';

// Créer une zone de sortie
const output = document.createElement('div');
output.innerHTML = '<h3>Exemple pratique de forkJoin:</h3>';
document.body.appendChild(output);

// Flux de données factices
const user$ = of({ id: 1, name: 'Taro Yamada' }).pipe(delay(2000));
const posts$ = of([{ id: 1, title: 'Post 1' }, { id: 2, title: 'Post 2' }]).pipe(delay(1500));
const weather$ = of({ temp: 22, condition: 'Ensoleillé' }).pipe(delay(1000));

// Message de chargement
const loading = document.createElement('div');
loading.textContent = 'Chargement des données...';
loading.style.color = 'blue';
output.appendChild(loading);

// Produire tout en une fois après que toutes les requêtes soient terminées
forkJoin({
  user: user$,
  posts: posts$,
  weather: weather$
}).subscribe(result => {
  output.removeChild(loading);

  const pre = document.createElement('pre');
  pre.textContent = JSON.stringify(result, null, 2);
  pre.style.background = '#f5f5f5';
  pre.style.padding = '10px';
  pre.style.borderRadius = '5px';
  output.appendChild(pre);

  const summary = document.createElement('div');
  summary.textContent = `Utilisateur: ${result.user.name}, Météo: ${result.weather.condition}, Posts: ${result.posts.length}`;
  output.appendChild(summary);
});
```

- Affichage du chargement en premier,
- Lorsque toutes les données sont disponibles, les résultats sont affichés ensemble.
