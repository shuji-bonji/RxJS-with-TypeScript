---
description: "combineLatestWith est un opérateur de combinaison RxJS qui combine l'Observable original avec les dernières valeurs d'autres Observables. Il est idéal pour les situations où vous souhaitez surveiller en permanence les dernières valeurs de plusieurs flux dépendants, comme la validation en temps réel des entrées de formulaire, la synchronisation de plusieurs états, les mises à jour en temps réel des résultats de calcul, etc. La version de l'opérateur pipeable est pratique pour une utilisation au sein de pipelines."
---

# combineLatestWith - combine les valeurs les plus récentes

L'opérateur `combineLatestWith` combine toutes les **dernières valeurs** de l'Observable original et de tout autre Observable spécifié.
Chaque fois qu'une nouvelle valeur est émise par l'un des Observable, un résultat est émis qui combine toutes les dernières valeurs.
C'est la version Pipeable Operator de la fonction Creation Function `combineLatest`.

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

// Exemples de sorties:
// A0 + B0
// A1 + B0
// A2 + B0
// A2 + B1
```

- Après que chaque Observable a émis **au moins une valeur**, la valeur combinée est émise.
- Chaque fois qu'une nouvelle valeur arrive d'un côté ou de l'autre, la dernière paire est réémise.

[🌐 Documentation officielle de RxJS - `combineLatestWith`](https://rxjs.dev/api/operators/combineLatestWith)

## 💡 Modèle d'utilisation typique.

- **Validation en temps réel des entrées de formulaire** : surveillance constante de l'état le plus récent de plusieurs champs.
- Synchronisation de plusieurs états dépendants** : combinaison de valeurs de configuration et d'entrées utilisateur.
- Mise à jour en temps réel des résultats de calcul** : calcul immédiat des résultats à partir de plusieurs valeurs d'entrée.

## 🧠 Exemples de codes pratiques (avec interface utilisateur)

Exemple de calcul du montant total en temps réel à partir des entrées de prix et de quantité.

```ts
import { fromEvent } from 'rxjs';
import { combineLatestWith, map, startWith } from 'rxjs';

// Création de zones de sortie
const output = document.createElement('div');
output.innerHTML = '<h3>combineLatestWith Exemples pratiques de:</h3>';
document.body.appendChild(output);

// Création de champs d'entrée
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

// de chaque entréeObservable
const price$ = fromEvent(priceInput, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value) || 0),
  startWith(100)
);

const quantity$ = fromEvent(quantityInput, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value) || 0),
  startWith(1)
);

// Calculé en combinant les dernières valeurs
price$
  .pipe(
    combineLatestWith(quantity$),
    map(([price, quantity]) => price * quantity)
  )
  .subscribe((total) => {
    result.innerHTML = `<strong>Montant total: ¥${total.toLocaleString()}</strong>`;
  });
```

- Lorsque vous entrez dans l'un ou l'autre champ, le total est **immédiatement recalculé** à partir des deux valeurs les plus récentes.
- La fonction `startWith()` est utilisée pour obtenir le résultat combiné depuis le début.

## 🔄 Différences avec la Creation Function `combineLatest`.

### Différences de base.

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

// Exemples de sorties:
// A0 + B0
// A1 + B0
// A2 + B0
// A2 + B1
```

### Exemples spécifiques d'utilisation

**Si vous souhaitez uniquement des combinaisons simples, la Creation Function est la meilleure solution**.


```ts
import { combineLatest, of } from 'rxjs';

const firstName$ = of('Taro');
const lastName$ = of('Yamada');
const age$ = of(30);

// Simple et facile à lire
combineLatest([firstName$, lastName$, age$]).subscribe(([first, last, age]) => {
  console.log(`${last} ${first}3 (${age}Âge)`);
});
// Sortie: Yamada Taro (30Âge)
```

**Si vous souhaitez ajouter un processus de conversion au flux principal, le Pipeable Operator est recommandé**.

```ts
import { fromEvent, interval } from 'rxjs';
import { combineLatestWith, map, startWith, debounceTime } from 'rxjs';

const searchInput = document.createElement('input');
searchInput.placeholder = 'Recherche de...';
document.body.appendChild(searchInput);

const categorySelect = document.createElement('select');
categorySelect.innerHTML = '<option>Tous les livres</option><option>Livres</option><option>DVD</option>';
document.body.appendChild(categorySelect);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Mainstream: Recherche de mots-clés
const searchTerm$ = fromEvent(searchInput, 'input').pipe(
  map(e => (e.target as HTMLInputElement).value),
  debounceTime(300),  // Après l'entrée300msAttendre
  startWith('')
);

// Sous-système: Sélection de la catégorie
const category$ = fromEvent(categorySelect, 'change').pipe(
  map(e => (e.target as HTMLSelectElement).value),
  startWith('Tous les livres')
);

// ✅ Pipeable OperatorÉdition - Complété en une seule fois
searchTerm$
  .pipe(
    map(term => term.toLowerCase()),  // Converti en minuscules
    combineLatestWith(category$),
    map(([term, category]) => ({
      term,
      category,
      timestamp: new Date().toLocaleTimeString()
    }))
  )
  .subscribe(result => {
    output.textContent = `Recherche de: "${result.term}" Catégorie: ${result.category} [${result.timestamp}]`;
  });

// ❌ Creation FunctionÉdition - Devenir redondant
import { combineLatest } from 'rxjs';
combineLatest([
  searchTerm$.pipe(map(term => term.toLowerCase())),
  category$
]).pipe(
  map(([term, category]) => ({
    term,
    category,
    timestamp: new Date().toLocaleTimeString()
  }))
).subscribe(result => {
  output.textContent = `Recherche de: "${result.term}" Catégorie: ${result.category} [${result.timestamp}]`;
});
```

**Lorsque vous combinez plusieurs valeurs de configuration**.

```ts
import { fromEvent } from 'rxjs';
import { combineLatestWith, map, startWith } from 'rxjs';

// Création d'un curseur
const redSlider = document.createElement('input');
redSlider.type = 'range';
redSlider.min = '0';
redSlider.max = '255';
redSlider.value = '255';
document.body.appendChild(document.createTextNode('Red: '));
document.body.appendChild(redSlider);
document.body.appendChild(document.createElement('br'));

const greenSlider = document.createElement('input');
greenSlider.type = 'range';
greenSlider.min = '0';
greenSlider.max = '255';
greenSlider.value = '0';
document.body.appendChild(document.createTextNode('Green: '));
document.body.appendChild(greenSlider);
document.body.appendChild(document.createElement('br'));

const blueSlider = document.createElement('input');
blueSlider.type = 'range';
blueSlider.min = '0';
blueSlider.max = '255';
blueSlider.value = '0';
document.body.appendChild(document.createTextNode('Blue: '));
document.body.appendChild(blueSlider);

const colorBox = document.createElement('div');
colorBox.style.width = '200px';
colorBox.style.height = '100px';
colorBox.style.marginTop = '10px';
colorBox.style.border = '1px solid #ccc';
document.body.appendChild(colorBox);

// Mainstream: Red
const red$ = fromEvent(redSlider, 'input').pipe(
  map(e => Number((e.target as HTMLInputElement).value)),
  startWith(255)
);

// ✅ Pipeable OperatorÉdition - RedCombiner d'autres couleurs comme couleurs principales
red$
  .pipe(
    combineLatestWith(
      fromEvent(greenSlider, 'input').pipe(
        map(e => Number((e.target as HTMLInputElement).value)),
        startWith(0)
      ),
      fromEvent(blueSlider, 'input').pipe(
        map(e => Number((e.target as HTMLInputElement).value)),
        startWith(0)
      )
    ),
    map(([r, g, b]) => `rgb(${r}, ${g}, ${b})`)
  )
  .subscribe(color => {
    colorBox.style.backgroundColor = color;
    colorBox.textContent = color;
    colorBox.style.display = 'flex';
    colorBox.style.alignItems = 'center';
    colorBox.style.justifyContent = 'center';
    colorBox.style.color = '#fff';
    colorBox.style.textShadow = '1px 1px 2px #000';
  });
```

### Résumé.

- **`combineLatest`** : idéal si vous voulez simplement combiner plusieurs flux.
- **combineLatestWith** : idéal si vous voulez combiner les dernières valeurs d'autres flux tout en transformant ou en traitant le flux principal.

## ⚠️ Notes.

### Pas de publication tant que les valeurs initiales ne sont pas disponibles.

Aucun résultat n'est émis tant que tous les Observable n'ont pas émis au moins une valeur.

```ts
import { interval, NEVER } from 'rxjs';
import { combineLatestWith, take } from 'rxjs';

interval(1000).pipe(
  take(3),
  combineLatestWith(NEVER)  // Aucune valeur émiseObservable
).subscribe(console.log);
// Pas de sortie (parce que)NEVERPas de sortie (parce que)
```

Ce problème peut être résolu en fournissant une valeur initiale avec `startWith()`.

```ts
import { interval, NEVER } from 'rxjs';
import { combineLatestWith, take, startWith } from 'rxjs';

interval(1000).pipe(
  take(3),
  combineLatestWith(NEVER.pipe(startWith(null)))
).subscribe(console.log);
// Sortie: [0, null] → [1, null] → [2, null]
```

### Attention aux rééditions fréquentes.

Si un flux émet fréquemment des valeurs, le résultat sera également réémis fréquemment.

```ts
import { interval } from 'rxjs';
import { combineLatestWith } from 'rxjs';

// 100msFlux émis chaque
const fast$ = interval(100);
const slow$ = interval(1000);

fast$.pipe(
  combineLatestWith(slow$)
).subscribe(console.log);
// slow$chaque fois qu'une valeur est émise,fast$est émise, elle est combinée avec la dernière valeur de
// → Les performances nécessitent une attention particulière
```

### Gestion des erreurs.

Si une erreur survient dans un Observable, tout le processus se termine par une erreur.

```ts
import { throwError, interval } from 'rxjs';
import { combineLatestWith, take, catchError } from 'rxjs';
import { of } from 'rxjs';

interval(1000).pipe(
  take(2),
  combineLatestWith(
    throwError(() => new Error('Des erreurs se produisent')).pipe(
      catchError((err: unknown) => of('Récupération'))
    )
  )
).subscribe({
  next: console.log,
  error: err => console.error(err.message)
});
// Sortie: [0, 'Récupération'] → [1, 'Récupération']
```

## 📚 Opérateurs apparentés.

- **[combineLatest](/fr/guide/creation-functions/combination/combineLatest)** - version de la Creation Function.
- **[withLatestFrom](/fr/guide/operators/combination/withLatestFrom)** - déclenché uniquement par le courant principal.
- **[zipWith](/fr/guide/operators/combination/zipWith)** - Associe des valeurs correspondantes.
