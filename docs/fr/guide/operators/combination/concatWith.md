---
description: "concatWith est un opérateur de combinaison RxJS qui combine séquentiellement d'autres Observables après la complétion de l'Observable original. Idéal pour le traitement séquentiel dans un pipeline, le traitement de suivi après complétion, le chargement progressif de données. Version pipeable operator pratique à utiliser dans les pipelines."
---

# concatWith - Concaténer en Séquence

L'opérateur `concatWith` **combine séquentiellement** les autres Observables spécifiés après que l'Observable original soit `complete`.
C'est la version Pipeable Operator de la Creation Function `concat`.

## 🔰 Syntaxe de base et utilisation

```ts
import { of, delay } from 'rxjs';
import { concatWith } from 'rxjs';

const obs1$ = of('A', 'B').pipe(delay(100));
const obs2$ = of('C', 'D').pipe(delay(100));
const obs3$ = of('E', 'F').pipe(delay(100));

obs1$
  .pipe(concatWith(obs2$, obs3$))
  .subscribe(console.log);

// Sortie: A → B → C → D → E → F
```

- `obs2$` commence après la complétion de `obs1$`, et `obs3$` commence après la complétion de `obs2$`.
- Peut être utilisé dans une chaîne `.pipe()`, facilitant la combinaison avec d'autres opérateurs.

[🌐 Documentation officielle RxJS - `concatWith`](https://rxjs.dev/api/operators/concatWith)


## 💡 Patterns d'utilisation typiques

- **Traitement séquentiel dans un pipeline** : Combiner séquentiellement des données supplémentaires à un flux transformé
- **Traitement de suivi après complétion** : Ajouter un nettoyage ou une notification après le traitement principal
- **Chargement progressif de données** : Récupérer séquentiellement des données supplémentaires après l'obtention des données initiales


## 🧠 Exemple de code pratique (avec UI)

Un exemple qui affiche les résultats de recherche principaux, puis affiche séquentiellement les articles recommandés.

```ts
import { of, delay } from 'rxjs';
import { concatWith, map } from 'rxjs';

// Création de la zone de sortie
const output = document.createElement('div');
output.innerHTML = '<h3>Exemple pratique de concatWith :</h3>';
document.body.appendChild(output);

// Résultats de recherche principaux
const searchResults$ = of('🔍 Résultat 1', '🔍 Résultat 2', '🔍 Résultat 3').pipe(
  delay(500)
);

// Recommandations 1
const recommendations1$ = of('💡 Produit recommandé A', '💡 Produit recommandé B').pipe(
  delay(300)
);

// Recommandations 2
const recommendations2$ = of('⭐ Produit populaire X', '⭐ Produit populaire Y').pipe(
  delay(300)
);

// Combiner séquentiellement et afficher
searchResults$
  .pipe(
    concatWith(recommendations1$, recommendations2$),
    map((value, index) => `${index + 1}. ${value}`)
  )
  .subscribe((value) => {
    const item = document.createElement('div');
    item.textContent = value;
    output.appendChild(item);
  });
```

- Les résultats de recherche s'affichent d'abord,
- Puis les produits recommandés s'affichent séquentiellement.
- Peut être combiné avec d'autres opérateurs comme `map` dans le pipeline.


## 🔄 Différence avec la Creation Function `concat`

### Différences de base

| | `concat` (Creation Function) | `concatWith` (Pipeable Operator) |
|:---|:---|:---|
| **Lieu d'utilisation** | Utilisé comme fonction indépendante | Utilisé dans la chaîne `.pipe()` |
| **Syntaxe** | `concat(obs1$, obs2$, obs3$)` | `obs1$.pipe(concatWith(obs2$, obs3$))` |
| **Premier flux** | Traité également | Traité comme flux principal |
| **Avantage** | Simple et lisible | Facile à combiner avec d'autres opérateurs |

### Exemples concrets de choix

**Pour une simple combinaison, la Creation Function est recommandée**

```ts
import { concat, of } from 'rxjs';

const part1$ = of('A', 'B');
const part2$ = of('C', 'D');
const part3$ = of('E', 'F');

// Simple et lisible
concat(part1$, part2$, part3$).subscribe(console.log);
// Sortie: A → B → C → D → E → F
```

**Quand une transformation est nécessaire en cours de route, le Pipeable Operator est recommandé**

```ts
import { of } from 'rxjs';
import { concatWith, map, filter } from 'rxjs';

const userData$ = of({ name: 'Alice', age: 30 }, { name: 'Bob', age: 25 });
const additionalData$ = of({ name: 'Charlie', age: 35 });

// ✅ Version Pipeable Operator - complète en un seul pipeline
userData$
  .pipe(
    filter(user => user.age >= 30),  // 30 ans et plus seulement
    map(user => user.name),          // Extraire le nom uniquement
    concatWith(
      additionalData$.pipe(map(user => user.name))
    )
  )
  .subscribe(console.log);
// Sortie: Alice → Charlie
```

**Pour ajouter un traitement de suivi au flux principal**

```ts
import { fromEvent, of } from 'rxjs';
import { concatWith, take, map } from 'rxjs';

// Création du bouton et de la zone de sortie
const button = document.createElement('button');
button.textContent = 'Cliquez 3 fois';
document.body.appendChild(button);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

const clicks$ = fromEvent(button, 'click');

// ✅ Version Pipeable Operator - naturel comme extension du flux principal
clicks$
  .pipe(
    take(3),                          // Obtenir les 3 premiers clics
    map(() => 'Cliqué'),
    concatWith(of('Terminé'))    // Message supplémentaire après complétion
  )
  .subscribe(message => {
    const div = document.createElement('div');
    div.textContent = message;
    output.appendChild(div);
  });
```

### Résumé

- **`concat`** : Optimal pour simplement combiner plusieurs flux
- **`concatWith`** : Optimal quand vous voulez ajouter des transformations au flux principal et ajouter des suites


## ⚠️ Points d'attention

### Délai dû à l'attente de complétion

L'Observable suivant ne démarre pas tant que l'original n'est pas terminé.

```ts
import { interval, of } from 'rxjs';
import { concatWith, take } from 'rxjs';

interval(1000).pipe(
  take(3),              // Terminer après 3
  concatWith(of('Terminé'))
).subscribe(console.log);
// Sortie: 0 → 1 → 2 → Terminé
```

### Gestion des erreurs

Si une erreur se produit dans l'Observable précédent, les Observables suivants ne sont pas exécutés.

```ts
import { throwError, of } from 'rxjs';
import { concatWith, catchError } from 'rxjs';

throwError(() => new Error('Erreur survenue'))
  .pipe(
    catchError((err: unknown) => of('Erreur récupérée')),
    concatWith(of('Traitement suivant'))
  )
  .subscribe(console.log);
// Sortie: Erreur récupérée → Traitement suivant
```


## 📚 Opérateurs associés

- **[concat](/fr/guide/creation-functions/combination/concat)** - Version Creation Function
- **[mergeWith](/fr/guide/operators/combination/mergeWith)** - Version Pipeable pour combinaison parallèle
- **[concatMap](/fr/guide/operators/transformation/concatMap)** - Mapping séquentiel de chaque valeur
