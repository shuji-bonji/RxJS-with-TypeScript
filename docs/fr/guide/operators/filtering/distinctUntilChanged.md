---
description: "L'opérateur distinctUntilChanged ignore les valeurs consécutives identiques et n'émet que les valeurs qui changent, permettant un traitement efficace des données."
---

# distinctUntilChanged - Ignorer doublons

L'opérateur `distinctUntilChanged` supprime les doublons lorsque des valeurs identiques sont émises consécutivement et n'émet que lorsque la valeur est différente de la précédente.


## 🔰 Syntaxe de base et utilisation

```ts
import { from } from 'rxjs';
import { distinctUntilChanged } from 'rxjs';

const numbers$ = from([1, 1, 2, 2, 3, 1, 2, 3]);

numbers$.pipe(
  distinctUntilChanged()
).subscribe(console.log);
// Sortie: 1, 2, 3, 1, 2, 3
```

- Les valeurs identiques à la précédente sont ignorées.
- Contrairement à un traitement par lot comme `Array.prototype.filter`, il **juge séquentiellement**.

[🌐 Documentation officielle RxJS - `distinctUntilChanged`](https://rxjs.dev/api/operators/distinctUntilChanged)

> [!WARNING] Attention en code de production
> L'exemple ci-dessus omet la désinscription de `fromEvent` pour simplifier l'explication. Dans du code réel, gérez explicitement le cycle de vie avec `takeUntil(destroy$)`, `take(N)`, ou `Subscription.unsubscribe()`. Détails : [Surmonter les difficultés : gestion du cycle de vie](/fr/guide/overcoming-difficulties/lifecycle-management.md)


## 💡 Patterns d'utilisation typiques

- Éviter les requêtes inutiles en détection d'entrée de formulaire quand la même valeur est saisie consécutivement
- Détection de changement dans les flux de capteurs ou d'événements
- Prévention de re-rendu UI inutile dans la gestion d'état


## 🧠 Exemple de code pratique (avec UI)

Une simulation où seules les entrées **différentes de la précédente** déclenchent une requête API dans une boîte de recherche.

```ts
import { fromEvent } from 'rxjs';
import { map, distinctUntilChanged } from 'rxjs';

// Création de la zone de sortie
const container = document.createElement('div');
document.body.appendChild(container);

const searchInput = document.createElement('input');
searchInput.type = 'text';
searchInput.placeholder = 'Entrer mot-clé de recherche';
container.appendChild(searchInput);

const resultArea = document.createElement('div');
resultArea.style.marginTop = '10px';
container.appendChild(resultArea);

// Flux d'entrée
fromEvent(searchInput, 'input')
  .pipe(
    distinctUntilChanged(),
    map((event) => (event.target as HTMLInputElement).value.trim())
  )
  .subscribe((keyword) => {
    resultArea.textContent = `Exécution de la recherche pour: ${keyword}`;
  });

```

- La requête n'est pas effectuée si la valeur saisie ne change pas.
- Peut être utilisé pour optimiser le traitement de recherche efficace et la communication API.
