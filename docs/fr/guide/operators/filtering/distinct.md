---
description: "L'opérateur distinct supprime toutes les valeurs en double et n'émet que les valeurs uniques qui n'ont jamais été émises. Il utilise un Set en interne pour mémoriser les valeurs déjà émises, donc attention avec les flux infinis."
---

# distinct - Supprimer toutes les valeurs en double

L'opérateur `distinct` surveille toutes les valeurs émises par un Observable et **n'émet que les valeurs qui n'ont jamais été émises**. Il utilise un Set en interne pour mémoriser les valeurs déjà émises.


## 🔰 Syntaxe de base et utilisation

```ts
import { of } from 'rxjs';
import { distinct } from 'rxjs';

const numbers$ = of(1, 2, 1, 3, 2, 4, 1, 5);

numbers$.pipe(
  distinct()
).subscribe(console.log);
// Sortie: 1, 2, 3, 4, 5
```

- Supprime les doublons sur l'ensemble du flux
- Une fois qu'une valeur a été émise, elle est ignorée chaque fois qu'elle réapparaît
- `distinctUntilChanged` supprime uniquement les doublons **consécutifs**, tandis que `distinct` supprime **tous** les doublons

[🌐 Documentation officielle RxJS - `distinct`](https://rxjs.dev/api/operators/distinct)


## 🆚 Différence avec distinctUntilChanged

```ts
import { of } from 'rxjs';
import { distinct, distinctUntilChanged } from 'rxjs';

const values$ = of(1, 2, 1, 2, 3, 1, 2, 3);

// distinctUntilChanged: supprime uniquement les doublons consécutifs
values$.pipe(
  distinctUntilChanged()
).subscribe(console.log);
// Sortie: 1, 2, 1, 2, 3, 1, 2, 3

// distinct: supprime tous les doublons
values$.pipe(
  distinct()
).subscribe(console.log);
// Sortie: 1, 2, 3
```

| Opérateur | Cible de suppression | Cas d'utilisation |
|---|---|---|
| `distinctUntilChanged` | Doublons consécutifs uniquement | Champs de saisie, données de capteur |
| `distinct` | Tous les doublons | Liste de valeurs uniques, liste d'IDs |


## 🎯 Personnalisation de la comparaison avec keySelector

Pour juger les doublons par une propriété spécifique d'un objet, utilisez une fonction `keySelector`.

```ts
import { of } from 'rxjs';
import { distinct } from 'rxjs';

interface User {
  id: number;
  name: string;
}

const users$ = of(
  { id: 1, name: 'Alice' } as User,
  { id: 2, name: 'Bob' } as User,
  { id: 1, name: 'Alice (updated)' } as User, // Même ID
  { id: 3, name: 'Charlie' } as User
);

users$.pipe(
  distinct(user => user.id) // Juger les doublons par ID
).subscribe(console.log);
// Sortie:
// { id: 1, name: 'Alice' }
// { id: 2, name: 'Bob' }
// { id: 3, name: 'Charlie' }
```


## 💡 Patterns d'utilisation typiques

1. **Récupération d'une liste d'IDs uniques**
   ```ts
   import { from } from 'rxjs';
   import { distinct, map } from 'rxjs';

   interface Order {
     orderId: string;
     userId: number;
     amount: number;
   }

   const orders$ = from([
     { orderId: 'A1', userId: 1, amount: 100 },
     { orderId: 'A2', userId: 2, amount: 200 },
     { orderId: 'A3', userId: 1, amount: 150 },
     { orderId: 'A4', userId: 3, amount: 300 }
   ] as Order[]);

   // Récupérer uniquement les IDs utilisateur uniques
   orders$.pipe(
     map(order => order.userId),
     distinct()
   ).subscribe(userId => {
     console.log(`User ID: ${userId}`);
   });
   // Sortie: 1, 2, 3
   ```

2. **Extraction des types d'événements uniques des logs**
   ```ts
   import { fromEvent, merge } from 'rxjs';
   import { map, distinct, take } from 'rxjs';

   // Création dynamique des éléments UI
   const container = document.createElement('div');
   document.body.appendChild(container);

   const button1 = document.createElement('button');
   button1.textContent = 'Bouton 1';
   container.appendChild(button1);

   const button2 = document.createElement('button');
   button2.textContent = 'Bouton 2';
   container.appendChild(button2);

   const input = document.createElement('input');
   input.placeholder = 'Entrez quelque chose';
   container.appendChild(input);

   const log = document.createElement('div');
   log.style.marginTop = '10px';
   container.appendChild(log);

   // Fusionner plusieurs flux d'événements et extraire les types uniques
   const events$ = merge(
     fromEvent(button1, 'click').pipe(map(() => 'button1-click')),
     fromEvent(button2, 'click').pipe(map(() => 'button2-click')),
     fromEvent(input, 'input').pipe(map(() => 'input-change'))
   );

   events$.pipe(
     distinct(),
     take(3) // Terminer quand les 3 types d'événements sont collectés
   ).subscribe({
     next: (eventType) => {
       log.textContent += `Événement unique: ${eventType}\n`;
       console.log(`Unique event: ${eventType}`);
     },
     complete: () => {
       log.textContent += 'Tous les types d\'événements ont été détectés';
     }
   });
   ```


## 🧠 Exemple de code pratique (Saisie de tags)

Un exemple d'UI qui supprime automatiquement les doublons des tags saisis par l'utilisateur.

```ts
import { fromEvent, Subject } from 'rxjs';
import { map, distinct, scan } from 'rxjs';

// Création des éléments UI
const container = document.createElement('div');
document.body.appendChild(container);

const tagInput = document.createElement('input');
tagInput.type = 'text';
tagInput.placeholder = 'Entrez un tag et appuyez sur Entrée';
container.appendChild(tagInput);

const tagList = document.createElement('ul');
tagList.style.marginTop = '10px';
container.appendChild(tagList);

// Flux d'ajout de tags
const tagSubject$ = new Subject<string>();

tagSubject$.pipe(
  map(tag => tag.trim().toLowerCase()),
  distinct() // Supprimer les tags en double
).subscribe(tag => {
  const li = document.createElement('li');
  li.textContent = tag;
  tagList.appendChild(li);
});

// Ajouter un tag avec la touche Entrée
fromEvent<KeyboardEvent>(tagInput, 'keydown').subscribe(event => {
  if (event.key === 'Enter') {
    const value = tagInput.value.trim();
    if (value) {
      tagSubject$.next(value);
      tagInput.value = '';
    }
  }
});
```

Ce code garantit que même si le même tag est saisi plusieurs fois, il n'est ajouté à la liste qu'une seule fois.


## ⚠️ Note sur l'utilisation mémoire

> [!WARNING]
> L'opérateur `distinct` utilise un **Set** en interne pour mémoriser toutes les valeurs déjà émises. Avec des flux infinis, cela peut causer des fuites de mémoire.

### Problème : Fuite de mémoire avec flux infini

```ts
import { interval } from 'rxjs';
import { distinct, map } from 'rxjs';

// ❌ Mauvais exemple: utiliser distinct avec un flux infini
interval(100).pipe(
  map(n => n % 10), // Cycle 0-9
  distinct() // Émet seulement les 10 premiers, puis continue à mémoriser
).subscribe(console.log);
// Sortie: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
// Après, rien n'est émis mais le Set est conservé
```

### Solution : Paramètre flushes pour vider le Set

```ts
import { interval, timer } from 'rxjs';
import { distinct, map } from 'rxjs';

// ✅ Bon exemple: vider le Set périodiquement
interval(100).pipe(
  map(n => n % 5),
  distinct(
    value => value,
    timer(1000) // Vider le Set chaque seconde
  )
).subscribe(console.log);
// 0, 1, 2, 3, 4 sont ré-émis chaque seconde
```

### Bonnes pratiques

1. **Utiliser avec des flux finis** : Réponses HTTP, conversions depuis tableaux, etc.
2. **Utiliser flushes** : Pour les flux infinis, vider périodiquement
3. **Considérer distinctUntilChanged** : Si vous ne devez supprimer que les doublons consécutifs


## 📋 Utilisation type-safe

Un exemple d'implémentation type-safe utilisant les génériques TypeScript.

```ts
import { Observable } from 'rxjs';
import { distinct, map } from 'rxjs';

interface Product {
  id: number;
  name: string;
  categoryId: number;
}

function getUniqueCategories(
  products$: Observable<Product>
): Observable<number> {
  return products$.pipe(
    distinct(product => product.categoryId)
  ).pipe(
    map(product => product.categoryId)
  );
}

// Exemple d'utilisation
import { of } from 'rxjs';

const products$ = of(
  { id: 1, name: 'Laptop', categoryId: 10 } as Product,
  { id: 2, name: 'Mouse', categoryId: 10 } as Product,
  { id: 3, name: 'Book', categoryId: 20 } as Product
);

getUniqueCategories(products$).subscribe(categoryId => {
  console.log(`Category ID: ${categoryId}`);
});
// Sortie: 10, 20
```


## 🎓 Résumé

### Quand utiliser distinct
- ✅ Quand une liste de valeurs uniques est nécessaire
- ✅ Pour supprimer les doublons dans des flux finis
- ✅ Création de listes d'IDs ou de catégories

### Quand utiliser distinctUntilChanged
- ✅ Pour supprimer uniquement les doublons consécutifs
- ✅ Détection de changement dans les champs de saisie
- ✅ Pour économiser la mémoire avec des flux infinis

### Points d'attention
- ⚠️ Utiliser le paramètre `flushes` pour éviter les fuites de mémoire avec des flux infinis
- ⚠️ Attention à l'utilisation mémoire avec beaucoup de valeurs uniques
- ⚠️ Si les performances sont importantes, surveillez la taille du Set


## 🚀 Prochaines étapes

- **[distinctUntilChanged](./distinctUntilChanged)** - Apprendre à supprimer uniquement les doublons consécutifs
- **[distinctUntilKeyChanged](./distinctUntilKeyChanged)** - Apprendre à comparer par clé d'objet
- **[filter](./filter)** - Apprendre le filtrage basé sur les conditions
- **[Exemples pratiques d'opérateurs de filtrage](./practical-use-cases)** - Apprendre des cas d'utilisation réels
