---
description: "L'opérateur tap est un opérateur utilitaire qui permet d'effectuer des effets de bord sans affecter la valeur du flux. Idéal pour le débogage avec sortie de journal, le contrôle des états de chargement, le suivi analytique, la surveillance des erreurs, etc. Les effets de bord peuvent être gérés dans du code déclaratif tout en maintenant la sécurité de type TypeScript."
---

# tap - Exécution des effets de bord

L'opérateur `tap` est utilisé pour « exécuter des effets de bord (side effects) sans modifier le flux ».
Idéal pour la journalisation, le débogage ou d'autres opérations qui n'affectent pas les valeurs.

## 🔰 Syntaxe et fonctionnement de base

Cet opérateur est utilisé dans les situations où seuls des effets de bord doivent être ajoutés sans modifier le flux de valeurs.

```ts
import { of, tap } from 'rxjs';

of(42).pipe(
  tap(value => console.log('tap:', value))
).subscribe();
// Sortie :
// tap: 42
```

Dans cet exemple, une valeur issue de `of(42)` est enregistrée lorsqu'elle passe par `tap`.
Le tap n'affecte pas le contenu du flux, car la valeur est transmise « telle quelle ».

[🌐 Documentation officielle RxJS - tap](https://rxjs.dev/api/index/function/tap)

## 💡 Cas d'utilisation typiques

`tap` est souvent utilisé dans les buts suivants :

- Débogage et journalisation
- Basculer entre les états de chargement
- Afficher les notifications toast
- Déclencher des mises à jour de l'interface utilisateur

```ts
import { of, tap, map } from 'rxjs';

of(Math.random()).pipe(
  tap(val => console.log('Valeur obtenue :', val)),
  map(n => n > 0.5 ? 'High' : 'Low'),
  tap(label => console.log('Label :', label))
).subscribe();
// Sortie :
// Valeur obtenue : 0.09909888881113504
// Label : Low
```


## 🧪 Exemple de code pratique (avec interface utilisateur)

Voici un exemple d'ajout de journaux au DOM à l'aide de tap.

```ts
import { of } from 'rxjs';
import { tap, map } from 'rxjs';

// Élément pour la sortie du journal
const logOutput = document.createElement('div');
document.body.appendChild(logOutput);

// Séquence de valeurs
of(1, 2, 3, 4, 5)
  .pipe(
    tap((val) => {
      console.log(`Valeur originale : ${val}`);

      // Ajouter le journal à l'interface utilisateur
      const logEntry = document.createElement('div');
      logEntry.textContent = `tap : valeur ${val} passée`;
      logEntry.style.color = '#666';
      logOutput.appendChild(logEntry);
    }),
    map((val) => val * 10),
    tap((val) => {
      console.log(`Valeur après conversion : ${val}`);

      // Ajouter le journal à l'interface utilisateur
      const logEntry = document.createElement('div');
      logEntry.textContent = `tap : valeur après conversion ${val}`;
      logEntry.style.color = '#090';
      logOutput.appendChild(logEntry);
    })
  )
  .subscribe((val) => {
    // Afficher le résultat final dans l'interface utilisateur
    const resultItem = document.createElement('div');
    resultItem.textContent = `Résultat : ${val}`;
    resultItem.style.fontWeight = 'bold';
    logOutput.appendChild(resultItem);
  });

```


## ✅ Résumé

- `tap` est un opérateur dédié à **l'insertion d'effets de bord**
- **Sortie de journal et mises à jour de l'interface utilisateur** sans changer le flux de valeurs
- Peut être combiné avec `finalize` et `catchError` pour un contrôle plus pratique
