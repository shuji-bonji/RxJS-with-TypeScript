---
description: "Ce guide explique comment mettre en place un modèle de développement d'apprentissage construit avec Vite, TypeScript et RxJS. Il dispose d'un rechargement à chaud, ce qui le rend idéal pour expérimenter le code et manipuler le DOM dans le navigateur, ainsi que pour le développement piloté par les tests à l'aide de Vitest."
---

# Mise en place de l'environnement d'apprentissage pratique

Cette page explique comment utiliser le modèle de développement [`RxJS-with-TypeScript-Starter-Kit`](https://github.com/shuji-bonji/rxjs-with-typescript-starter-kit), qui vous permet de tester instantanément le code RxJS localement plutôt que dans un navigateur.

## Caractéristiques

- Configuration simple : Vite + TypeScript + RxJS
- Prise en charge du rechargement à chaud (exécuter `npm run dev` pour tester immédiatement)
- Environnement de développement local prenant en charge la manipulation du DOM et les tests
- Développement piloté par les tests (TDD) possible avec Vitest

## Utilisation

Cloner et configurer en utilisant les commandes suivantes :

```bash
git clone https://github.com/shuji-bonji/rxjs-with-typescript-starter-kit.git
cd rxjs-with-typescript-starter-kit
npm install
npm run dev
```

Votre navigateur se lancera automatiquement et exécutera le code écrit dans `src/main.ts`.

## Exemple d'utilisation

Réécrivez le fichier `src/main.ts` existant comme suit.

```ts
// src/main.ts
import { interval } from 'rxjs';
import { take } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

interval(1000).pipe(take(5)).subscribe((val) => {
  const p = document.createElement('p');
  p.textContent = `Count: ${val}`;
  output.appendChild(p);
});
```

### Accès à localhost
L'URL sera affichée sous la forme `http://localhost:5174/`. Accédez à cette URL pour voir les résultats.
Pour vérifier les résultats de `console.log()`, utilisez la console de vos outils de développement.

```sh
% npm run dev

> rxjs-with-typescript-starter-kit@0.0.0 dev
> vite

Port 5173 is in use, trying another one...

  VITE v6.3.1  ready in 107 ms

  ➜  Local:   http://localhost:5174/
  ➜  Network: use --host to expose
  ➜  press h + enter to show help
```

## Utilisations recommandées

- Expérimenter avec les Observables / Subjects / Opérateurs
- Apprendre l'interface utilisateur réactive combinée avec le DOM
- Pratiquer l'implémentation de tests marble (`vitest` + `TestScheduler`)
- Environnement de base pour stocker vos propres extraits de code

## Liens

🔗 Modèle ici → [RxJS-with-TypeScript-Starter-Kit](https://github.com/shuji-bonji/rxjs-with-typescript-starter-kit)
Voir `README.md` pour plus de détails.
