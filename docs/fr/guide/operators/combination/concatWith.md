---
description: "ConcatWith est un opérateur de jointure RxJS qui joint d'autres Observables en séquence après l'achèvement de l'Observable d'origine. Il est idéal pour les situations où vous souhaitez ajouter un traitement ultérieur en tant qu'extension du flux principal, comme le traitement séquentiel dans un pipeline, le traitement de suivi après l'achèvement, le chargement de données par étapes, etc. La version de l'opérateur pipe est pratique pour une utilisation au sein d'un pipeline."
---

# concatWith - concaténer des flux en séquence

L'opérateur `concatWith` concatène **séquentiellement** les autres Observables spécifiés après que l'Observable original soit `complet`.
C'est la version Pipeable Operator de Creation Function's `concat`.

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

- `obs1$` se termine avant que `obs2$` ne commence, `obs2$` se termine avant que `obs3$` ne commence.
- Peut être utilisé dans une chaîne `.pipe()`, ce qui le rend plus facile à combiner avec d'autres opérateurs.

[🌐 Official RxJS documentation - `concatWith`](https://rxjs.dev/api/operators/concatWith)

## 💡 Modèle d'utilisation typique.

- **Traitement séquentiel dans le pipeline** : les données supplémentaires sont concaténées dans le flux converti en séquence.
- Traitement de suivi après l'achèvement** : ajout de nettoyage et de notifications après l'achèvement du traitement principal.
- Chargement échelonné des données** : acquisition séquentielle de données supplémentaires après l'acquisition initiale des données.

## 🧠 Exemples de codes pratiques (avec interface utilisateur)

Exemple d'affichage des principaux résultats de recherche, suivis d'une séquence d'éléments recommandés connexes.

```ts
import { of, delay } from 'rxjs';
import { concatWith, map } from 'rxjs';

// Création d'une zone de sortie
const output = document.createElement('div');
output.innerHTML = '<h3>concatWith Exemples pratiques de:</h3>';
document.body.appendChild(output);

// Résultats de la recherche principale
const searchResults$ = of('🔍 Résultats de la recherche1', '🔍 Résultats de la recherche2', '🔍 Résultats de la recherche3').pipe(
  delay(500)
);

// Éléments recommandés1
const recommendations1$ = of('💡 Éléments recommandésA', '💡 Éléments recommandésB').pipe(
  delay(300)
);

// Éléments recommandés2
const recommendations2$ = of('⭐ Articles populairesX', '⭐ Articles populairesY').pipe(
  delay(300)
);

// Combinés et affichés dans l'ordre
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

- Les résultats de la recherche sont affichés en premier,
- les éléments recommandés sont ensuite affichés dans l'ordre.
- Peut être utilisé en combinaison avec d'autres opérateurs tels que `map` dans le pipeline.

## 🔄 Différences avec la Creation Function `concat`.

### Différences de base.

|  | `concat` (Creation Function) | `concatWith` (Pipeable Operator) |
|---|---|---|
| **Où utilisé** | Utilisée comme fonction autonome | Utilisée dans la chaîne `.pipe()`. |
| **Comment écrire | `concat(obs1$, obs2$, obs3$)` | `obs1$.pipe(concatWith(obs2$, obs3$))` |
| **Premier flux**. | Traiter tous les flux comme égaux | Traiter comme le flux principal |
| **Avantages**. | Simple et facile à lire | Facile à combiner avec d'autres opérateurs |

### Exemples spécifiques d'utilisation

**Si vous souhaitez effectuer une simple jointure, la Creation Function est la meilleure solution**.

```ts
import { concat, of } from 'rxjs';

const part1$ = of('A', 'B');
const part2$ = of('C', 'D');
const part3$ = of('E', 'F');

// Simple et facile à lire
concat(part1$, part2$, part3$).subscribe(console.log);
// Sortie: A → B → C → D → E → F
```

**Si vous avez besoin de convertir au milieu, Pipeable Operator est recommandé**.

```ts
import { of } from 'rxjs';
import { concatWith, map, filter } from 'rxjs';

const userData$ = of({ name: 'Alice', age: 30 }, { name: 'Bob', age: 25 });
const additionalData$ = of({ name: 'Charlie', age: 35 });

// ❌ Creation FunctionÉdition - Redondante
import { concat } from 'rxjs';
concat(
  userData$.pipe(
    filter(user => user.age >= 30),
    map(user => user.name)
  ),
  additionalData$.pipe(map(user => user.name))
).subscribe(console.log);

// ✅ Pipeable OperatorÉdition - Complète en une seule fois
userData$
  .pipe(
    filter(user => user.age >= 30),  // 30Uniquement pour les personnes âgées de 18 ans et plus
    map(user => user.name),          // Extraire uniquement les noms
    concatWith(
      additionalData$.pipe(map(user => user.name))
    )
  )
  .subscribe(console.log);
// Sortie: Alice → Charlie
```

**Si vous souhaitez ajouter un traitement ultérieur au flux principal**.

```ts
import { fromEvent, of } from 'rxjs';
import { concatWith, take, map } from 'rxjs';

// Créer un bouton et une zone de sortie
const button = document.createElement('button');
button.textContent = '3Cliquer une seule fois';
document.body.appendChild(button);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

const clicks$ = fromEvent(button, 'click');

// ✅ Pipeable OperatorÉdition - Naturel en tant qu'extension du flux principal
clicks$
  .pipe(
    take(3),                          // Premiers3Obtenir des clics
    map(() => 'Clics'),
    concatWith(of('Terminé'))    // Message supplémentaire après l'achèvement
  )
  .subscribe(message => {
    const div = document.createElement('div');
    div.textContent = message;
    output.appendChild(div);
  });

// Le même comportement peut êtreCreation FunctionS'il est écrit dans une édition...
// ❌ Creation FunctionÉdition - Le courant principal doit être écrit séparément
import { concat } from 'rxjs';
concat(
  clicks$.pipe(
    take(3),
    map(() => 'Clics')
  ),
  of('Terminé')
).subscribe(console.log);
```

### Résumé.

- **`concat`** : idéal pour la fusion simple de plusieurs flux.
- **`concatWith`** : idéal si vous voulez ajouter des flux ultérieurs tout en transformant ou en traitant le flux principal.

## ⚠️ Notes.

### Retards causés par l'attente de l'achèvement de l'opération

L'Observable suivant ne démarrera pas tant que l'Observable initial ne sera pas terminé.

```ts
import { interval, of } from 'rxjs';
import { concatWith, take } from 'rxjs';

interval(1000).pipe(
  take(3),              // 3Complété en une seule fois
  concatWith(of('Complété'))
).subscribe(console.log);
// Sortie: 0 → 1 → 2 → Complété
```

### Gestion des erreurs.

Si une erreur survient dans l'Observable précédent, l'Observable suivant ne sera pas exécuté.

```ts
import { throwError, of } from 'rxjs';
import { concatWith, catchError } from 'rxjs';

throwError(() => new Error('Une erreur s'est produite'))
  .pipe(
    catchError((err: unknown) => of('Récupération de l'erreur')),
    concatWith(of('Processus suivant'))
  )
  .subscribe(console.log);
// Sortie: Récupération de l'erreur → Processus suivant
```

## 📚 Opérateurs apparentés.

- **[concat](/fr/guide/creation-functions/combination/concat)** - Version de Creation Function
- **[mergeWith](/fr/guide/operators/combination/mergeWith)** - Version canalisable pour fusionner en parallèle
- **[concatMap](/fr/guide/operators/transformation/concatMap)** - mappage séquentiel de valeurs individuelles
