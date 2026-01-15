---
description: "combineLatestWith est un opérateur de combinaison RxJS qui combine et émet les dernières valeurs de l'Observable original et d'autres Observables. Idéal pour la validation de formulaire en temps réel, la synchronisation d'états multiples, la mise à jour de calculs en temps réel. Version pipeable operator pratique à utiliser dans les pipelines."
---

# combineLatestWith - Dernières valeurs combinées

L'opérateur `combineLatestWith` **émet ensemble les dernières valeurs** de l'Observable original et des autres Observables spécifiés.
Chaque fois qu'un Observable émet une nouvelle valeur, le résultat combiné de toutes les dernières valeurs est émis.
C'est la version Pipeable Operator de la Creation Function `combineLatest`.

## 🔰 Syntaxe de base et utilisation

```ts
import { interval } from 'rxjs';
import { combineLatestWith, map, take } from 'rxjs';

const source1$ = interval(1000).pipe(
  map(val => `A${val}`),
  take(3)
);

const source2$ = interval(1500).pipe(
  map(val => `B${val}`),
  take(2)
);

source1$
  .pipe(combineLatestWith(source2$))
  .subscribe(([val1, val2]) => {
    console.log(`${val1} + ${val2}`);
  });

// Exemple de sortie:
// A0 + B0
// A1 + B0
// A2 + B0
// A2 + B1
```

- Les valeurs combinées sont émises **après que chaque Observable a émis au moins une valeur**.
- Chaque fois qu'une nouvelle valeur arrive de l'un des côtés, la dernière paire est ré-émise.

[🌐 Documentation officielle RxJS - `combineLatestWith`](https://rxjs.dev/api/operators/combineLatestWith)


## 💡 Patterns d'utilisation typiques

- **Validation de formulaire en temps réel** : Surveiller constamment le dernier état de plusieurs champs
- **Synchronisation d'états dépendants multiples** : Combinaison de valeurs de configuration et d'entrées utilisateur
- **Mise à jour de résultats de calcul en temps réel** : Calcul immédiat à partir de plusieurs valeurs d'entrée


## 🧠 Exemple de code pratique (avec UI)

Un exemple qui calcule le montant total en temps réel à partir des entrées de prix et de quantité.

```ts
import { fromEvent } from 'rxjs';
import { combineLatestWith, map, startWith } from 'rxjs';

// Création de la zone de sortie
const output = document.createElement('div');
output.innerHTML = '<h3>Exemple pratique de combineLatestWith :</h3>';
document.body.appendChild(output);

// Création des champs de saisie
const priceInput = document.createElement('input');
priceInput.type = 'number';
priceInput.placeholder = 'Prix unitaire';
priceInput.value = '100';
document.body.appendChild(priceInput);

const quantityInput = document.createElement('input');
quantityInput.type = 'number';
quantityInput.placeholder = 'Quantité';
quantityInput.value = '1';
document.body.appendChild(quantityInput);

// Zone d'affichage du résultat
const result = document.createElement('div');
result.style.fontSize = '20px';
result.style.marginTop = '10px';
document.body.appendChild(result);

// Observable pour chaque entrée
const price$ = fromEvent(priceInput, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value) || 0),
  startWith(100)
);

const quantity$ = fromEvent(quantityInput, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value) || 0),
  startWith(1)
);

// Combiner les dernières valeurs et calculer
price$
  .pipe(
    combineLatestWith(quantity$),
    map(([price, quantity]) => price * quantity)
  )
  .subscribe((total) => {
    result.innerHTML = `<strong>Montant total: ¥${total.toLocaleString()}</strong>`;
  });
```

- Saisir dans l'un ou l'autre champ **recalcule immédiatement le total à partir des 2 dernières valeurs**.
- En utilisant `startWith()`, nous pouvons obtenir un résultat combiné dès le départ.


## 🔄 Différence avec la Creation Function `combineLatest`

### Différences de base

| | `combineLatest` (Creation Function) | `combineLatestWith` (Pipeable Operator) |
|:---|:---|:---|
| **Lieu d'utilisation** | Utilisé comme fonction indépendante | Utilisé dans la chaîne `.pipe()` |
| **Syntaxe** | `combineLatest([obs1$, obs2$])` | `obs1$.pipe(combineLatestWith(obs2$))` |
| **Premier flux** | Traité également | Traité comme flux principal |
| **Valeur de retour** | Tableau `[val1, val2]` | Tuple `[val1, val2]` |
| **Avantage** | Simple et lisible | Facile à combiner avec d'autres opérateurs |

### Exemples concrets de choix

**Pour une simple combinaison, la Creation Function est recommandée**

```ts
import { combineLatest, of } from 'rxjs';

const firstName$ = of('Jean');
const lastName$ = of('Dupont');
const age$ = of(30);

// Simple et lisible
combineLatest([firstName$, lastName$, age$]).subscribe(([first, last, age]) => {
  console.log(`${first} ${last} (${age} ans)`);
});
// Sortie: Jean Dupont (30 ans)
```

**Pour ajouter des transformations au flux principal, le Pipeable Operator est recommandé**

```ts
import { fromEvent, interval } from 'rxjs';
import { combineLatestWith, map, startWith, debounceTime } from 'rxjs';

const searchInput = document.createElement('input');
searchInput.placeholder = 'Recherche...';
document.body.appendChild(searchInput);

const categorySelect = document.createElement('select');
categorySelect.innerHTML = '<option>Tous</option><option>Livres</option><option>DVD</option>';
document.body.appendChild(categorySelect);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Flux principal: terme de recherche
const searchTerm$ = fromEvent(searchInput, 'input').pipe(
  map(e => (e.target as HTMLInputElement).value),
  debounceTime(300),  // Attendre 300ms après saisie
  startWith('')
);

// Flux secondaire: sélection de catégorie
const category$ = fromEvent(categorySelect, 'change').pipe(
  map(e => (e.target as HTMLSelectElement).value),
  startWith('Tous')
);

// ✅ Version Pipeable Operator - complète en un seul pipeline
searchTerm$
  .pipe(
    map(term => term.toLowerCase()),  // Convertir en minuscules
    combineLatestWith(category$),
    map(([term, category]) => ({
      term,
      category,
      timestamp: new Date().toLocaleTimeString()
    }))
  )
  .subscribe(result => {
    output.textContent = `Recherche: "${result.term}" Catégorie: ${result.category} [${result.timestamp}]`;
  });
```

### Résumé

- **`combineLatest`** : Optimal pour simplement combiner plusieurs flux
- **`combineLatestWith`** : Optimal quand vous voulez ajouter des transformations au flux principal tout en combinant les dernières valeurs d'autres flux


## ⚠️ Points d'attention

### Pas d'émission tant que les valeurs initiales ne sont pas alignées

Aucun résultat n'est émis tant que tous les Observables n'ont pas émis au moins une valeur.

```ts
import { interval, NEVER } from 'rxjs';
import { combineLatestWith, take } from 'rxjs';

interval(1000).pipe(
  take(3),
  combineLatestWith(NEVER)  // Observable qui n'émet jamais
).subscribe(console.log);
// Pas de sortie (car NEVER n'émet pas de valeur)
```

Cela peut être résolu en donnant une valeur initiale avec `startWith()`.

```ts
import { interval, NEVER } from 'rxjs';
import { combineLatestWith, take, startWith } from 'rxjs';

interval(1000).pipe(
  take(3),
  combineLatestWith(NEVER.pipe(startWith(null)))
).subscribe(console.log);
// Sortie: [0, null] → [1, null] → [2, null]
```

### Attention aux ré-émissions fréquentes

Si l'un des flux émet fréquemment, le résultat sera également ré-émis fréquemment.

### Gestion des erreurs

Si une erreur se produit dans l'un des Observables, l'ensemble se termine en erreur.


## 📚 Opérateurs associés

- **[combineLatest](/fr/guide/creation-functions/combination/combineLatest)** - Version Creation Function
- **[withLatestFrom](/fr/guide/operators/combination/withLatestFrom)** - Seul le flux principal déclenche
- **[zipWith](/fr/guide/operators/combination/zipWith)** - Apparie les valeurs correspondantes
